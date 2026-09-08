package Hart;

import Vector::*;
import ConfigReg::*;
import RegIf::*;
import Decode::*;
import Muldiv::*;
import HartRegs::*;

// RV32I[M] 机器态核心。三级：取指 / 译码执行 / 写回，**停顿而不旁路**。
//
// 为什么先三级：100 MHz、55nm 下没有任何理由为频率加级数，真正决定级数的是
// 存储延迟，而那要等缓存挂上去、在仿真里量出停顿代价再说。BSV 的规则结构让
// 后加级数比 Verilog 便宜得多，所以这不是不可逆的选择。
//
// 取指与访存各一个 RegManager。契约当初写的是两个 Server，本意是「解耦、
// 容忍变长延迟」——RegManager 是同一件事的扁平形态，而 E23 量过 Server 在
// 小模块上要贵 39.3%，且 dma 已经在用 RegManager，同一个库里不该有两套发起口。
//
// 一条写法上的规矩：**每个寄存器在一条规则里只写一处**。分支里各写各的会让
// bsc 判成并行冲突（G0004），因为它把每个 `<=` 当成一次独立的方法调用。
// 所以下面一律先算出「下一拍是什么」，规则末尾再写一次。

typedef struct {
  Bool mul;
  Bool smode;
} HartCfg;

typedef enum { Fetch, Wait, Exec, Mem, Muls, CsrRd, CsrWr }
  Stage deriving (Bits, Eq, FShow);

// 平台送进来的东西：三根中断线与本核的编号。收在一个子接口里，
// 生成的顶层照 emit.pins 原样透传，跟其它 IP 一个形状。
interface HartPins;
  (* always_ready, always_enabled, prefix = "" *)
  method Action irq((* port = "msip" *) Bool msip,
                    (* port = "mtip" *) Bool mtip,
                    (* port = "meip" *) Bool meip);
  (* always_ready, always_enabled, prefix = "" *)
  method Action hartid((* port = "hartid" *) Bit#(32) v);
  // 拉高就不取指。调试要停核，测试台要在核跑之前把程序装进内存，
  // 两件事是同一个需求。
  (* always_ready, always_enabled, prefix = "" *)
  method Action halt((* port = "halt" *) Bool v);
endinterface

interface HartIfc#(numeric type aw, numeric type dw);
  interface RegManager#(32, 32) imem;
  interface RegManager#(32, 32) dmem;
  interface HartPins            pins;
endinterface

module mkHart#(HartCfg cfg)(HartIfc#(aw, dw))
    // CSR 空间的宽度由 ISA 定死：12 位地址、32 位数据。参数留着是为了让
    // 生成的包装层照统一的形状写，这两条 proviso 把它们钉在唯一合法的值上。
    provisos (Add#(aw, 0, 12), Add#(dw, 0, 32));

  HartRegsIfc#(aw, dw) csrf <- mkHartRegs(HartRegsCfg { smode: cfg.smode });
  MuldivIfc md <- (cfg.mul ? mkMuldiv : mkMuldivNone);

  Vector#(32, Reg#(Bit#(32))) rf <- replicateM(mkConfigReg(0));
  Reg#(Bit#(32)) pc    <- mkConfigReg('h8000_0000);
  Reg#(Stage)    st    <- mkConfigReg(Fetch);
  Reg#(Bit#(32)) instr <- mkReg(0);
  Reg#(Decoded)  dec   <- mkReg(unpack(0));
  Reg#(Bit#(32)) memAd <- mkReg(0);
  Reg#(Bit#(32)) csrNv <- mkReg(0);
  Reg#(Bool)     csrWr <- mkReg(False);

  Reg#(Bool) iValid <- mkConfigReg(False);
  Reg#(Bool) dValid <- mkConfigReg(False);
  Reg#(RegReq#(32, 32)) dReq <- mkReg(unpack(0));

  Wire#(Bool)        iRdy  <- mkBypassWire;
  Wire#(Bool)        dRdy  <- mkBypassWire;
  Wire#(Bool)        iRspV <- mkBypassWire;
  Wire#(RegRsp#(32)) iRspX <- mkBypassWire;
  Wire#(Bool)        dRspV <- mkBypassWire;
  Wire#(RegRsp#(32)) dRspX <- mkBypassWire;

  Wire#(Bool) msipIn <- mkBypassWire;
  Wire#(Bool) mtipIn <- mkBypassWire;
  Wire#(Bool) meipIn <- mkBypassWire;
  // 退休计数不能跟 CSR 访问写在同一条规则里：minstret 是软硬双写的 CReg，
  // 两个端口不许在一条规则里同时用（G0004）。改成发个脉冲，计数单列一处。
  PulseWire retire <- mkPulseWire;
  Reg#(Bit#(32))  hid   <- mkReg(0);
  Wire#(Bit#(32)) hidIn <- mkBypassWire;
  Wire#(Bool)     halted <- mkBypassWire;

  function Bit#(32) rd(Bit#(5) i) = (i == 0) ? 0 : rf[i];

  function Action wr(Bit#(5) i, Bit#(32) v) = action
    if (i != 0) rf[i] <= v;
  endaction;

  function Bit#(32) alu(AluOp o, Bit#(32) a, Bit#(32) b);
    case (o)
      OpAdd:  return a + b;
      OpSub:  return a - b;
      OpSll:  return a << b[4:0];
      OpSlt:  return signedLT(unpack(a), unpack(b)) ? 1 : 0;
      OpSltu: return (a < b) ? 1 : 0;
      OpXor:  return a ^ b;
      OpSrl:  return a >> b[4:0];
      OpSra:  return pack(signedShiftRight(unpack(a), b[4:0]));
      OpOr:   return a | b;
      default: return a & b;
    endcase
  endfunction

  function Bool branchTaken(Bit#(3) f, Bit#(32) a, Bit#(32) b);
    case (f)
      0: return a == b;
      1: return a != b;
      4: return signedLT(unpack(a), unpack(b));
      5: return signedGE(unpack(a), unpack(b));
      6: return a < b;
      default: return a >= b;
    endcase
  endfunction

  Bool irqPending = csrf.mstatus_mie == 1 &&
      ((csrf.mie_msie == 1 && msipIn) || (csrf.mie_mtie == 1 && mtipIn)
       || (csrf.mie_meie == 1 && meipIn));

  Bit#(31) irqCode = (csrf.mie_meie == 1 && meipIn) ? 11
                   : (csrf.mie_mtie == 1 && mtipIn) ? 7 : 3;

  // 两件事必须分成两条规则：驱动没有存储的 CSR 要排在 CSR 访问**之前**
  // （访问要读它们），更新计数器要排在**之后**（访问读的是旧值）。
  // 写一条里就首尾相接，bsc 判 CSR 规则永不触发。
  rule platform;
    csrf.misa_in(32'h4000_0100 | (cfg.mul ? 32'h0000_1000 : 0));   // RV32I[M]
    csrf.mvendorid_in(0);
    csrf.marchid_in(0);
    csrf.mhartid_in(hid);
    csrf.mip_msip_in(msipIn ? 1 : 0);
    csrf.mip_mtip_in(mtipIn ? 1 : 0);
    csrf.mip_meip_in(meipIn ? 1 : 0);
    hid <= hidIn;
  endrule

  rule tick;
    csrf.mcycle_in(csrf.mcycle + 1);
    if (retire) csrf.minstret_in(csrf.minstret + 1);
  endrule

  // 陷入的现场保存与跳转。写 pc 与 st 由调用处统一做，这里只碰 CSR。
  function Action enterTrap(Bit#(31) code, Bool isIrq, Bit#(32) tval) = action
    csrf.mepc_in(pc);
    csrf.mcause_code_in(code);
    csrf.mcause_intr_in(isIrq ? 1 : 0);
    csrf.mtval_in(tval);
    csrf.mstatus_mpie_in(csrf.mstatus_mie);
    csrf.mstatus_mie_in(0);
    csrf.mstatus_mpp_in(3);
  endaction;

  function Bit#(32) trapTarget(Bit#(31) code, Bool isIrq);
    Bit#(32) base = {csrf.mtvec_base, 2'b00};
    // 向量模式下中断按编号偏移，异常一律走基址
    return (csrf.mtvec_mode == 1 && isIrq)
         ? base + (zeroExtend(code) << 2) : base;
  endfunction

  rule doFetch (st == Fetch && !halted);
    if (irqPending) begin
      enterTrap(irqCode, True, 0);
      pc <= trapTarget(irqCode, True);
    end else begin
      // 举手不放，等授予。授予与响应同拍到，所以只看 iRspV 就够。
      iValid <= True;
      st <= Wait;
    end
  endrule

  rule doWait (st == Wait && iRspV);
    iValid <= False;
    instr  <= iRspX.rdata;
    dec    <= decode(iRspX.rdata, cfg.mul);
    st     <= Exec;
  endrule

  rule doExec (st == Exec);
    let d = dec;
    Bit#(32) a = rd(d.rs1);
    Bit#(32) b = rd(d.rs2);
    Bit#(32) next = pc + 4;

    // 先把「下一拍是什么」算出来，末尾统一写
    Bit#(32) nPc  = next;
    Stage    nSt  = Fetch;
    Bool     bump = True;          // 这条指令这一拍就退休了吗

    case (d.kind)
      Reg: if (d.isMul) begin
             md.start(d.alu, a, b);
             nSt  = Muls;
             nPc  = pc;
             bump = False;
           end else
             wr(d.rd, alu(d.alu, a, b));
      Imm:   wr(d.rd, alu(d.alu, a, d.imm));
      Lui:   wr(d.rd, d.imm);
      Auipc: wr(d.rd, pc + d.imm);
      Jal:   begin wr(d.rd, next); nPc = pc + d.imm; end
      Jalr:  begin wr(d.rd, next); nPc = (a + d.imm) & ~32'h1; end
      Branch: if (branchTaken(d.fn3, a, b)) nPc = pc + d.imm;
      Load: begin
        Bit#(32) ad = a + d.imm;
        memAd  <= ad;
        dReq   <= RegReq { addr: ad & ~32'h3, write: False,
                           wdata: 0, wstrb: 4'hF };
        dValid <= True;
        nSt = Mem; nPc = pc; bump = False;
      end
      Store: begin
        Bit#(32) ad = a + d.imm;
        Bit#(2)  lo = ad[1:0];
        Bit#(4)  strb = case (d.fn3)
                          0: (4'b0001 << lo);
                          1: (4'b0011 << lo);
                          default: 4'b1111;
                        endcase;
        memAd  <= ad;
        dReq   <= RegReq { addr: ad & ~32'h3, write: True,
                           wdata: b << {lo, 3'b000}, wstrb: strb };
        dValid <= True;
        nSt = Mem; nPc = pc; bump = False;
      end
      Csr: begin
        nSt = CsrRd; nPc = pc; bump = False;
      end
      Sys: begin
        if (d.imm == 32'h000) begin                        // ecall
          enterTrap(11, False, 0);
          nPc = trapTarget(11, False);
        end else if (d.imm == 32'h001) begin               // ebreak
          enterTrap(3, False, 0);
          nPc = trapTarget(3, False);
        end else if (d.imm == 32'h302) begin               // mret
          nPc = csrf.mepc;
          csrf.mstatus_mie_in(csrf.mstatus_mpie);
          csrf.mstatus_mpie_in(1);
        end
        // fence 与 wfi 当空操作：本实现顺序执行、不休眠
      end
      default: begin                                        // 非法指令
        enterTrap(2, False, instr);
        nPc = trapTarget(2, False);
      end
    endcase

    pc <= nPc;
    st <= nSt;
    if (bump) retire.send();
  endrule

  rule doMul (st == Muls && md.done);
    wr(dec.rd, md.result);
    pc <= pc + 4;
    st <= Fetch;
    retire.send();
  endrule

  rule doMem (st == Mem && dRspV);
    dValid <= False;
    let d = dec;
    if (!dReq.write) begin
      Bit#(2)  lo = memAd[1:0];
      Bit#(32) w  = dRspX.rdata >> {lo, 3'b000};
      Bit#(32) v  = case (d.fn3)
                      0: signExtend(w[7:0]);
                      1: signExtend(w[15:0]);
                      4: zeroExtend(w[7:0]);
                      5: zeroExtend(w[15:0]);
                      default: w;
                    endcase;
      wr(d.rd, v);
    end
    pc <= pc + 4;
    st <= Fetch;
    retire.send();
  endrule

  // CSR 真的分两拍：一条规则里 access 只能调一次，而 csrrs/csrrc 要拿旧值算新值
  rule doCsrRead (st == CsrRd);
    let d = dec;
    Bit#(32) src = d.csrImm ? d.imm : rd(d.rs1);
    let old <- csrf.regs.access(RegReq { addr: truncate(d.csr), write: False,
                                         wdata: 0, wstrb: 4'hF });
    wr(d.rd, old.rdata);
    csrNv <= case (d.csrOp)
               CsrRw: src;
               CsrRs: (old.rdata | src);
               default: (old.rdata & ~src);
             endcase;
    // rs1 为 0 的 set/clear 是纯读，不该产生写副作用
    Bool doWrite = (d.csrOp == CsrRw) || (d.csrImm ? d.imm != 0 : d.rs1 != 0);
    csrWr <= doWrite;
    st    <= doWrite ? CsrWr : Fetch;
    if (!doWrite) begin
      pc <= pc + 4;
      retire.send();
    end
  endrule

  rule doCsrWrite (st == CsrWr);
    let _ <- csrf.regs.access(RegReq { addr: truncate(dec.csr), write: True,
                                       wdata: csrNv, wstrb: 4'hF });
    pc <= pc + 4;
    st <= Fetch;
    retire.send();
  endrule

  interface RegManager imem;
    method Bool valid = iValid;
    method RegReq#(32, 32) req = RegReq { addr: pc, write: False,
                                          wdata: 0, wstrb: 4'hF };
    method Action ready(Bool v); iRdy._write(v); endmethod
    method Action resp(Bool v, RegRsp#(32) x);
      iRspV._write(v);
      iRspX._write(x);
    endmethod
  endinterface

  interface RegManager dmem;
    method Bool valid = dValid;
    method RegReq#(32, 32) req = dReq;
    method Action ready(Bool v); dRdy._write(v); endmethod
    method Action resp(Bool v, RegRsp#(32) x);
      dRspV._write(v);
      dRspX._write(x);
    endmethod
  endinterface

  interface HartPins pins;
    method Action irq(Bool msip, Bool mtip, Bool meip);
      msipIn._write(msip);
      mtipIn._write(mtip);
      meipIn._write(meip);
    endmethod
    method Action hartid(Bit#(32) v); hidIn._write(v); endmethod
    method Action halt(Bool v); halted._write(v); endmethod
  endinterface
endmodule

endpackage
