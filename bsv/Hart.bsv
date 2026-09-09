package Hart;

import Vector::*;
import ConfigReg::*;
import RegIf::*;
import Decode::*;
import Muldiv::*;
import HartRegs::*;

// RV32I[M] 核心，M 态必备、S/U 两态由 smode 开关决定。三级：取指 / 译码执行 / 写回，**停顿而不旁路**。
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

typedef enum { Fetch, Exec, Mem, Muls, CsrRd, CsrWr }
  Stage deriving (Bits, Eq, FShow);

// 平台内部的三根中断线单列一个子接口：装配要把 aclint 的 mtip 与 plic 的 eip
// 接到这里，而一个 always_enabled 方法不能既被片内规则调、又从顶层透传出去。
// 分开之后，装配里接了 irq 就不透传 irq，hartid 与 halt 照旧露到顶层——
// 那两个本来就是板级的：编号是接线定的，停核是调试要的。
interface HartIrq;
  (* always_ready, always_enabled, prefix = "" *)
  method Action irq((* port = "msip" *) Bool msip,
                    (* port = "mtip" *) Bool mtip,
                    (* port = "meip" *) Bool meip);
endinterface

interface HartPins;
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
  interface HartIrq             irq;
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

  // 当前特权级，编码同 mstatus.mpp：0 用户、1 监管者、3 机器。
  // smode 关掉时它恒为 3，写它的地方全被 setPriv 折没，寄存器自然不留。
  Reg#(Bit#(2))  privR <- mkConfigReg(2'b11);

  // 取指的举手是组合的：请求当拍出去、响应当拍回来，于是取指只占一拍。
  // 原来用寄存器举手，请求要下一拍才出现在总线上，白搭一拍。
  // 存储真有延迟时这只是「举手不放」，行为不变——手一直举着直到授予。
  //
  // 取指还与执行**重叠**：执行当前指令的这一拍就把顺序下一条要回来，
  // 地址对得上就直接接着执行，直线代码一条指令一拍。
  // 不猜分支方向，只赌「下一条是 pc + 4」；跳走了对不上，回 Fetch 重取，
  // 代价是本来也要付的那一拍。
  //
  // 不设缓冲寄存器：存储一拍就答的时候缓冲填得太晚，同一拍里检查不到，
  // 白占面积。存储真慢起来，慢的也不是这一拍。
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

  Bit#(2) priv = cfg.smode ? privR : 2'b11;

  function Action setPriv(Bit#(2) v) = action
    if (cfg.smode) privR <= v;
  endaction;

  // 委托只对「在 M 以下的特权级发生」的陷入生效，M 态自己的陷入永远留在 M。
  function Bool deleg(Bit#(31) code, Bool isIrq) =
    cfg.smode && priv != 2'b11 &&
    ((isIrq ? csrf.mideleg : csrf.medeleg)[code[4:0]] == 1);

  // 全局开关：陷入目标级别高于当前级别时无条件接受，等于当前级别才看使能位。
  Bool mEn = (priv != 2'b11) || csrf.mstatus_mie == 1;
  Bool sEn = cfg.smode &&
             ((priv == 2'b00) || (priv == 2'b01 && csrf.mstatus_sie == 1));

  Bool msi = csrf.mie_msie == 1 && msipIn;
  Bool mti = csrf.mie_mtie == 1 && mtipIn;
  Bool mei = csrf.mie_meie == 1 && meipIn;
  // S 级软件中断的来源就是 mip.SSIP 那一位本身，没有外来的线
  Bool ssi = cfg.smode && csrf.mie_ssie == 1 && csrf.mip_ssip == 1
             && csrf.mideleg[1] == 1;

  Bool irqPending = (mEn && (msi || mti || mei)) || (sEn && ssi);

  // 链接处只看上一拍锁下来的这份。直接读 irqPending 会让写 CSR 那条规则
  // 同时碰 mstatus 那个 CReg 的两个端口（G0004）——CSR 写本来就在改使能位。
  // 晚一拍取中断是合法的：中断本来就异步，多退休一条指令不改变语义。
  Reg#(Bool) irqSeen <- mkConfigReg(False);

  Bit#(31) irqCode = (mEn && mei) ? 11 : (mEn && mti) ? 7
                   : (mEn && msi) ? 3 : 1;

  // 两件事必须分成两条规则：驱动没有存储的 CSR 要排在 CSR 访问**之前**
  // （访问要读它们），更新计数器要排在**之后**（访问读的是旧值）。
  // 写一条里就首尾相接，bsc 判 CSR 规则永不触发。
  rule platform;
    csrf.misa_in(32'h4000_0100 | (cfg.mul ? 32'h0000_1000 : 0)
                 | (cfg.smode ? 32'h0014_0000 : 0));   // RV32I[M][SU]
    csrf.mvendorid_in(0);
    csrf.marchid_in(0);
    csrf.mhartid_in(hid);
    csrf.mip_msip_in(msipIn ? 1 : 0);
    csrf.mip_mtip_in(mtipIn ? 1 : 0);
    csrf.mip_meip_in(meipIn ? 1 : 0);
    hid <= hidIn;
  endrule

  rule latchIrq;
    irqSeen <= irqPending;
  endrule

  rule tick;
    csrf.mcycle_in(csrf.mcycle + 1);
    if (retire) csrf.minstret_in(csrf.minstret + 1);
  endrule

  // 陷入的现场保存与跳转。写 pc 与 st 由调用处统一做，这里只碰 CSR。
  function Action enterTrap(Bit#(31) code, Bool isIrq, Bit#(32) tval) = action
    if (deleg(code, isIrq)) begin
      csrf.sepc_in(pc);
      csrf.scause_code_in(code);
      csrf.scause_intr_in(isIrq ? 1 : 0);
      csrf.stval_in(tval);
      csrf.mstatus_spie_in(csrf.mstatus_sie);
      csrf.mstatus_sie_in(0);
      csrf.mstatus_spp_in(priv[0]);
      setPriv(2'b01);
    end else begin
      csrf.mepc_in(pc);
      csrf.mcause_code_in(code);
      csrf.mcause_intr_in(isIrq ? 1 : 0);
      csrf.mtval_in(tval);
      csrf.mstatus_mpie_in(csrf.mstatus_mie);
      csrf.mstatus_mie_in(0);
      csrf.mstatus_mpp_in(priv);
      setPriv(2'b11);
    end
  endaction;

  function Bit#(32) trapTarget(Bit#(31) code, Bool isIrq);
    Bool s = deleg(code, isIrq);
    Bit#(32) base = s ? {csrf.stvec_base, 2'b00} : {csrf.mtvec_base, 2'b00};
    Bit#(2)  mode = s ? csrf.stvec_mode : csrf.mtvec_mode;
    // 向量模式下中断按编号偏移，异常一律走基址
    return (mode == 1 && isIrq) ? base + (zeroExtend(code) << 2) : base;
  endfunction

  Bit#(32) pfAddr  = pc + 4;

  // 这笔响应是哪个地址的。原来直接拿当拍发出去的地址判，那假设「响应与请求
  // 同拍」——只对组合应答的目标成立。存储慢一拍（缓存就是），回来的是上一笔，
  // 链接会把上一笔的数当成下一条指令接上。
  //
  // 记的时机是「被收下但没同拍答复」。组合目标上两件事同拍发生，什么也记不下，
  // 判的还是当拍地址，行为一字不变。
  Reg#(Bool)     inFlt <- mkConfigReg(False);
  Reg#(Bit#(32)) fltAd <- mkConfigReg(0);
  Bool     needNow = st == Fetch && !halted && !irqPending;
  // 除法一位一拍要磨三十几拍，那几拍里不举手——请求举着也没处放，
  // 只是白占总线仲裁。末拍再举，链接照样接得上。
  Bool     wantPf  = st != Fetch && !halted && (st != Muls || md.done);

  // 一条指令的最后一拍都走这里：顺序下一条这拍已经取回来了就直接接上，
  // 省掉回 Fetch 的那一拍。访存、乘除、读写 CSR 的末拍 pc 还停在本条上，
  // pfAddr 正好是下一条；跳走了地址对不上，照旧回 Fetch 重取。
  Bit#(32) askAd = needNow ? pc : pfAddr;
  Bit#(32) rspAd = inFlt ? fltAd : askAd;
  function Action advance(Bit#(32) nPc) = action
    // 待决中断必须让链接断开：链接跳过的正是 Fetch 那一拍，而中断只在那里
    // 检查。不断开的话一段直线代码能把中断拖到段尾，S 软件中断的用例就是
    // 这么暴露出来的（自己写 sip.SSIP，却先把后面两条执行完了）。
    Bool chain = iRspV && (rspAd == nPc) && !irqSeen;
    if (chain) begin
      instr <= iRspX.rdata;
      dec   <= decode(iRspX.rdata, cfg.mul);
    end
    pc <= nPc;
    st <= chain ? Exec : Fetch;
  endaction;


  rule track;
    if ((needNow || wantPf) && iRdy && !iRspV) begin
      inFlt <= True;
      fltAd <= askAd;
    end else if (iRspV)
      inFlt <= False;
  endrule

  rule doTrapEntry (st == Fetch && !halted && irqPending);
    enterTrap(irqCode, True, 0);
    pc <= trapTarget(irqCode, True);
  endrule

  // 响应回来了才走。没回来就停在 Fetch，手一直举着。
  rule doFetch (st == Fetch && !halted && !irqPending && iRspV
                && rspAd == pc);
    instr <= iRspX.rdata;
    dec   <= decode(iRspX.rdata, cfg.mul);
    st    <= Exec;
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
      Csr: if (priv < d.csr[9:8]) begin
             // 地址的第 9、8 位写明了这个 CSR 属于哪一级，够不着就是非法指令
             enterTrap(2, False, instr);
             nPc = trapTarget(2, False);
           end else begin
             nSt = CsrRd; nPc = pc; bump = False;
           end
      Sys: begin
        // ecall 的编号说的是「从哪一级喊的」：8 用户、9 监管者、11 机器
        Bit#(31) ec = (priv == 2'b00) ? 8 : (priv == 2'b01) ? 9 : 11;
        if (d.imm == 32'h000) begin                        // ecall
          enterTrap(ec, False, 0);
          nPc = trapTarget(ec, False);
        end else if (d.imm == 32'h001) begin               // ebreak
          enterTrap(3, False, 0);
          nPc = trapTarget(3, False);
        end else if (d.imm == 32'h302) begin               // mret
          nPc = csrf.mepc;
          csrf.mstatus_mie_in(csrf.mstatus_mpie);
          csrf.mstatus_mpie_in(1);
          setPriv(csrf.mstatus_mpp);
          // 返回后 mpp 退到实现支持的最低级：有 U 就退到 U，没有就还是 M
          csrf.mstatus_mpp_in(cfg.smode ? 2'b00 : 2'b11);
        end else if (cfg.smode && d.imm == 32'h102) begin  // sret
          nPc = csrf.sepc;
          csrf.mstatus_sie_in(csrf.mstatus_spie);
          csrf.mstatus_spie_in(1);
          setPriv(zeroExtend(csrf.mstatus_spp));
          csrf.mstatus_spp_in(0);
        end
        // fence 与 wfi 当空操作：本实现顺序执行、不休眠
      end
      default: begin                                        // 非法指令
        enterTrap(2, False, instr);
        nPc = trapTarget(2, False);
      end
    endcase

    if (nSt == Fetch) advance(nPc);
    else begin
      pc <= nPc;
      st <= nSt;
    end
    if (bump) retire.send();
  endrule

  rule doMul (st == Muls && md.done);
    wr(dec.rd, md.result);
    advance(pc + 4);
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
    advance(pc + 4);
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
    if (doWrite) st <= CsrWr;
    else begin
      // CSR 访问之后不链接，规规矩矩过一趟 Fetch。写 CSR 改的正是中断使能与
      // 待决位，而锁存下来的那份还是旧的；链过去就会把刚开起来的中断漏掉。
      pc <= pc + 4;
      st <= Fetch;
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
    method Bool valid = needNow || wantPf;
    method RegReq#(32, 32) req = RegReq { addr: askAd, write: False,
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

  interface HartIrq irq;
    method Action irq(Bool msip, Bool mtip, Bool meip);
      msipIn._write(msip);
      mtipIn._write(mtip);
      meipIn._write(meip);
    endmethod
  endinterface

  interface HartPins pins;
    method Action hartid(Bit#(32) v); hidIn._write(v); endmethod
    method Action halt(Bool v); halted._write(v); endmethod
  endinterface
endmodule

endpackage
