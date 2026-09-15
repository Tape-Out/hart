package Hart;

import Vector::*;
import ConfigReg::*;
import RegIf::*;
import Decode::*;
import Muldiv::*;
import HartRegs::*;
import Mmu::*;
import Rvfi::*;
import RvfiPins::*;

// 恒定的只读桩：rvfi 关掉时占位，综合器整片消掉
function Reg#(t) roReg(t v) =
  interface Reg;
    method t _read = v;
    method Action _write(t x) = noAction;
  endinterface;

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
  Bool mmu;
  Bool rvfi;
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
                    (* port = "meip" *) Bool meip,
                    // 这一根是边沿不是电平：SSWI 写一次送一拍，清零归软件
                    (* port = "ssip_set" *) Bool ssipSet);
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
  // 形式验证用的 RVFI 端口（riscv-formal rvfi.rst）；rvfi 关掉时恒为零
  (* prefix = "" *) interface RvfiPins rvfi;
endinterface

module mkHart#(HartCfg cfg)(HartIfc#(aw, dw))
    // CSR 空间的宽度由 ISA 定死：12 位地址、32 位数据。参数留着是为了让
    // 生成的包装层照统一的形状写，这两条 proviso 把它们钉在唯一合法的值上。
    provisos (Add#(aw, 0, 12), Add#(dw, 0, 32));

  HartRegsIfc#(aw, dw) csrf <- mkHartRegs(HartRegsCfg { smode: cfg.smode, mmu: cfg.mmu });
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

  // 流水线这一侧包成契约的形状，好让 MMU 用现成的 mkPipe 插进中间。
  // MMU 上游是**会停顿的目标**、下游是发起方，正是这两个契约的用处。
  PulseWire sfenceP <- mkPulseWire;
  // 清表的脉冲要先落一拍再交给 MMU。直接给的话：执行那一级发线、MMU 的配置
  // 规则读线，于是配置要排在执行之后；而配置写的线又喂给下游的请求，要排在
  // 取指之前——绕一圈闭合成环（G0009），bsc 直接把规则丢掉。
  // 晚一拍无妨：那条指令本来就要走完，下一次取指才用得上新的表。
  Reg#(Bool) sfenceR <- mkReg(False);
  Wire#(Bool) iPf <- mkDWire(False);
  Wire#(Bool) dPf <- mkDWire(False);

  Wire#(Bool) msipIn <- mkBypassWire;
  Wire#(Bool) ssipSetIn <- mkBypassWire;
  Wire#(Bool) mtipIn <- mkBypassWire;
  Wire#(Bool) meipIn <- mkBypassWire;
  // 退休计数不能跟 CSR 访问写在同一条规则里：minstret 是软硬双写的 CReg，
  // 两个端口不许在一条规则里同时用（G0004）。改成发个脉冲，计数单列一处。
  PulseWire retire <- mkPulseWire;
  // RVFI：出记录的规则把这一条放到线上，rvLatch 一条规则统一编号、打一拍输出。
  // trapped 说这一拍进了陷入，下一条出记录的指令就是处理程序的头一条（rvfi_intr）
  RWire#(Rvfi)   rvW      <- mkRWire;
  PulseWire      trapped  <- mkPulseWire;
  Reg#(Rvfi)     rvR      = roReg(idle);
  Reg#(Bit#(64)) rvOrder  = roReg(0);
  Reg#(Bool)     rvIntr   = roReg(False);
  // CSR 读拍读出的旧值与 rs1 的值：写拍出记录时 rd 已经改过，rd 与 rs1 同号时不能重读
  Reg#(Bit#(32)) rvCsrOld = roReg(0);
  Reg#(Bit#(32)) rvCsrRs1 = roReg(0);
  if (cfg.rvfi) begin
    rvR      <- mkReg(idle);
    rvOrder  <- mkReg(0);
    rvIntr   <- mkReg(False);
    rvCsrOld <- mkReg(0);
    rvCsrRs1 <- mkReg(0);
  end
  Reg#(Bit#(32))  hid   <- mkReg(0);
  Wire#(Bit#(32)) hidIn <- mkBypassWire;
  Wire#(Bool)     halted <- mkBypassWire;

  function Bit#(32) rd(Bit#(5) i) = (i == 0) ? 0 : rf[i];

  function Action wr(Bit#(5) i, Bit#(32) v) = action
    if (i != 0) rf[i] <= v;
  endaction;

  // 访存口一笔只碰一个字，跨到下一个字的字节会被丢掉，所以半字访问要偶地址、整字访问要低两位为零。
  // 不对齐就在翻译之前报地址不对齐，特权规范 3.1.15：*Implementations that never support misaligned
  // accesses can unconditionally raise the misaligned-address exception without performing address
  // translation or protection checks.*
  function Bool misaligned(Bit#(3) fn3, Bit#(32) ad) = case (fn3[1:0])
                                                        1: ad[0] != 0;
                                                        2: ad[1:0] != 0;
                                                        default: False;
                                                      endcase;

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
  // S 态里被 M 截获的三件事（3.1.6.5）：TVM 管 satp 与 sfence.vma，TSR 管 sret
  Bool tvmTrap = cfg.smode && priv == 2'b01 && csrf.mstatus_tvm == 1;
  Bool tsrTrap = cfg.smode && priv == 2'b01 && csrf.mstatus_tsr == 1;

  function Action setPriv(Bit#(2) v) = action
    if (cfg.smode) privR <= v;
  endaction;

  // 一条记录的公共部分：指令字、执行前后的 pc、特权级、rs1/rs2 执行前的值
  function Rvfi rvBase(Bit#(32) nextPc, Bool trap);
    Rvfi r = idle;
    r.valid    = True;
    r.insn     = instr;
    r.trap     = trap;
    r.mode     = priv;
    r.ixl      = 1;
    r.pc_rdata = pc;
    r.pc_wdata = nextPc;
    return readRs(dec.rs1, rd(dec.rs1), dec.rs2, rd(dec.rs2), r);
  endfunction

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
  // S 级软件中断看的是 mip.SSIP 那一位：软件能自己写，SSWI 也能置位
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

  // 单列一条：置位走 CReg 的高端口，得排在 CSR 访问之后。放进 platform
  // 就与「platform 要排在 CSR 访问之前」首尾相接。
  rule sswi;
    csrf.mip_ssip_set(ssipSetIn ? 1 : 0);
  endrule

  rule tick;
    csrf.mcycle_in(csrf.mcycle + 1);
    if (retire) csrf.minstret_in(csrf.minstret + 1);
  endrule

  rule rvLatch (cfg.rvfi);
    Rvfi r = fromMaybe(idle, rvW.wget);
    if (isValid(rvW.wget)) begin
      r.order = rvOrder;
      r.intr  = rvIntr;
      rvOrder <= rvOrder + 1;
      // 陷入的那一条自己不带标记，它后面的那一条带
      rvIntr  <= trapped;
    end else if (trapped)
      rvIntr <= True;
    rvR <= r;
  endrule

  // 陷入的现场保存与跳转。写 pc 与 st 由调用处统一做，这里只碰 CSR。
  function Action enterTrap(Bit#(31) code, Bool isIrq, Bit#(32) tval) = action
    if (deleg(code, isIrq)) begin
      csrf.sepc_in(pc[31:2]);
      csrf.scause_code_in(code);
      csrf.scause_intr_in(isIrq ? 1 : 0);
      csrf.stval_in(tval);
      csrf.mstatus_spie_in(csrf.mstatus_sie);
      csrf.mstatus_sie_in(0);
      csrf.mstatus_spp_in(priv[0]);
      setPriv(2'b01);
    end else begin
      csrf.mepc_in(pc[31:2]);
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
    Bool chain = iRspV && !iRspX.err && (rspAd == nPc) && !irqSeen;
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
    trapped.send();
  endrule

  // 响应回来了才走。没回来就停在 Fetch，手一直举着。
  rule doFetch (st == Fetch && !halted && !irqPending && iRspV
                && rspAd == pc);
    if (iRspX.err) begin
      // 取指答的是错。缺页与总线上的访问错异常号不同（12 与 1），
      // MMU 那根线说的就是「这一次是缺页」。原来 err 根本没人看——
      // 取错了的字会被当成指令解码执行。
      Bit#(31) code = iPf ? 12 : 1;
      enterTrap(code, False, pc);
      pc <= trapTarget(code, False);
      trapped.send();
    end else begin
      instr <= iRspX.rdata;
      dec   <= decode(iRspX.rdata, cfg.mul);
      st    <= Exec;
    end
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
    Bool     isTrap = False;       // 这条指令陷入了（仍出 RVFI 记录，但不算 minstret）

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
      // 没有 C 扩展，IALIGN 是 32：目标低两位非零就在跳转这一条上陷入，rd 不写。特权规范 3.1.15：
      // *Instruction address misaligned exceptions are raised by control-flow instructions with
      // misaligned targets, rather than by the act of fetching an instruction.*
      Jal, Jalr: begin
        Bit#(32) t = (d.kind == Jal) ? pc + d.imm : (a + d.imm) & ~32'h1;
        if (t[1:0] == 0) begin
          wr(d.rd, next);
          nPc = t;
        end else begin
          enterTrap(0, False, t);
          nPc = trapTarget(0, False);
          isTrap = True;
        end
      end
      Branch: if (branchTaken(d.fn3, a, b)) begin
        Bit#(32) t = pc + d.imm;
        if (t[1:0] == 0) nPc = t;
        else begin
          enterTrap(0, False, t);
          nPc = trapTarget(0, False);
          isTrap = True;
        end
      end
      Load: begin
        Bit#(32) ad = a + d.imm;
        if (misaligned(d.fn3, ad)) begin
          enterTrap(4, False, ad);
          nPc = trapTarget(4, False);
          isTrap = True;
        end else begin
          memAd  <= ad;
          dReq   <= RegReq { addr: ad & ~32'h3, write: False,
                             wdata: 0, wstrb: 4'hF };
          dValid <= True;
          nSt = Mem; nPc = pc; bump = False;
        end
      end
      Store: begin
        Bit#(32) ad = a + d.imm;
        Bit#(2)  lo = ad[1:0];
        Bit#(4)  strb = case (d.fn3)
                          0: (4'b0001 << lo);
                          1: (4'b0011 << lo);
                          default: 4'b1111;
                        endcase;
        if (misaligned(d.fn3, ad)) begin
          enterTrap(6, False, ad);
          nPc = trapTarget(6, False);
          isTrap = True;
        end else begin
          memAd  <= ad;
          dReq   <= RegReq { addr: ad & ~32'h3, write: True,
                             wdata: b << {lo, 3'b000}, wstrb: strb };
          dValid <= True;
          nSt = Mem; nPc = pc; bump = False;
        end
      end
      Csr: if (priv < d.csr[9:8] || (tvmTrap && d.csr == 'h180)) begin
             // 地址的第 9、8 位写明了这个 CSR 属于哪一级，够不着就是非法指令
             enterTrap(2, False, instr);
             nPc = trapTarget(2, False);
             isTrap = True;
           end else begin
             nSt = CsrRd; nPc = pc; bump = False;
           end
      Sys: begin
        // ecall 的编号说的是「从哪一级喊的」：8 用户、9 监管者、11 机器
        Bit#(31) ec = (priv == 2'b00) ? 8 : (priv == 2'b01) ? 9 : 11;
        if (d.imm == 32'h000) begin                        // ecall
          enterTrap(ec, False, 0);
          nPc = trapTarget(ec, False);
          isTrap = True;
        end else if (d.imm == 32'h001) begin               // ebreak
          enterTrap(3, False, 0);
          nPc = trapTarget(3, False);
          isTrap = True;
        end else if (d.imm == 32'h302 && priv == 2'b11) begin  // mret
          nPc = {csrf.mepc, 2'b00};
          csrf.mstatus_mie_in(csrf.mstatus_mpie);
          csrf.mstatus_mpie_in(1);
          setPriv(csrf.mstatus_mpp);
          if (csrf.mstatus_mpp != 2'b11) csrf.mstatus_mprv_in(0);
          // 返回后 mpp 退到实现支持的最低级：有 U 就退到 U，没有就还是 M
          csrf.mstatus_mpp_in(cfg.smode ? 2'b00 : 2'b11);
        end else if (cfg.smode && d.imm == 32'h102 && priv != 2'b00 && !tsrTrap) begin  // sret
          nPc = {csrf.sepc, 2'b00};
          csrf.mstatus_sie_in(csrf.mstatus_spie);
          csrf.mstatus_spie_in(1);
          setPriv(zeroExtend(csrf.mstatus_spp));
          // 回到比 M 低的特权级就清 MPRV（3.1.6.3）；sret 总是回到比 M 低的级
          csrf.mstatus_mprv_in(0);
          csrf.mstatus_spp_in(0);
        end else if (cfg.smode && d.imm[11:5] == 7'b0001001 && priv != 2'b00 && !tvmTrap) begin  // sfence.vma
          // 本版不分 ASID 也不分地址，一律整表清掉——规范允许比要求更狠地清
          // （特权规范 4.2.1：实现可以把 sfence.vma 当成清空全部翻译缓存）。
          // 没有 mmu 就没有东西可清，指令本身照样合法。
          if (cfg.mmu) sfenceP.send();
        end else if (d.imm == 32'h105 && !(cfg.smode && priv != 2'b11 && csrf.mstatus_tw == 1)) begin
          // wfi 当空操作：本实现顺序执行、不休眠。立刻返回算「有界时间内完成」，
          // 所以 TW 为零时 U 态执行它也不必判非法（3.3.3）；TW 置上时超时上限取 0，
          // 低于 M 的特权级一律非法（3.1.6.5 明写上限可以恒为 0）。
        end else begin
          // 够不着的特权指令与不认识的 SYSTEM 指令一律非法。原来落空成空操作：
          // U 态一条 mret 就跳到 mepc，还把特权级设成 mpp。
          enterTrap(2, False, instr);
          nPc = trapTarget(2, False);
          isTrap = True;
        end
      end
      default: begin                                        // 非法指令
        enterTrap(2, False, instr);
        nPc = trapTarget(2, False);
        isTrap = True;
      end
    endcase

    if (nSt == Fetch) advance(nPc);
    else begin
      pc <= nPc;
      st <= nSt;
    end
    // minstret 只算退休且没陷入的：特权规范 3.3.1 *As ECALL and EBREAK cause synchronous exceptions,
    // they are not considered to retire, and should not increment the minstret CSR.*
    if (bump && !isTrap) retire.send();
    if (isTrap) trapped.send();
    if (cfg.rvfi && bump) begin
      // rd 写的值在 RVFI 这边另算一遍：不去动上面那条数据通路，rvfi 关掉时这一段整片消掉
      Bit#(32) rdv  = 0;
      Bool     wrRd = False;
      case (d.kind)
        Reg:   begin wrRd = True; rdv = alu(d.alu, a, b); end
        Imm:   begin wrRd = True; rdv = alu(d.alu, a, d.imm); end
        Lui:   begin wrRd = True; rdv = d.imm; end
        Auipc: begin wrRd = True; rdv = pc + d.imm; end
        Jal:   begin wrRd = True; rdv = next; end
        Jalr:  begin wrRd = True; rdv = next; end
      endcase
      rvW.wset(writeRd((wrRd && !isTrap) ? d.rd : 0, rdv, rvBase(nPc, isTrap)));
    end
  endrule

  rule doMul (st == Muls && md.done);
    wr(dec.rd, md.result);
    advance(pc + 4);
    retire.send();
    if (cfg.rvfi) rvW.wset(writeRd(dec.rd, md.result, rvBase(pc + 4, False)));
  endrule

  rule doMem (st == Mem && dRspV);
    dValid <= False;
    let d = dec;
    if (dRspX.err) begin
      // 读缺页 13、写缺页 15；总线上的访问错分别是 5 与 7。
      // tval 要的是出错的那个**虚**地址（特权规范 4.3.2）。
      Bit#(31) code = dReq.write ? (dPf ? 15 : 7) : (dPf ? 13 : 5);
      enterTrap(code, False, memAd);
      pc <= trapTarget(code, False);
      st <= Fetch;
      trapped.send();
      if (cfg.rvfi) rvW.wset(access(memAd & ~32'h3, 0, 0, 0, 0, rvBase(trapTarget(code, False), True)));
    end else begin
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
        if (cfg.rvfi) rvW.wset(writeRd(d.rd, v, access(memAd & ~32'h3, 4'hF, dRspX.rdata, 0, 0, rvBase(pc + 4, False))));
      end else if (cfg.rvfi)
        rvW.wset(access(memAd & ~32'h3, 0, 0, dReq.wstrb, dReq.wdata, rvBase(pc + 4, False)));
      advance(pc + 4);
      retire.send();
    end
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
    if (cfg.rvfi) begin
      rvCsrOld <= old.rdata;
      rvCsrRs1 <= rd(d.rs1);
    end
    if (doWrite) st <= CsrWr;
    else begin
      // CSR 访问之后不链接，规规矩矩过一趟 Fetch。写 CSR 改的正是中断使能与
      // 待决位，而锁存下来的那份还是旧的；链过去就会把刚开起来的中断漏掉。
      pc <= pc + 4;
      st <= Fetch;
      retire.send();
      if (cfg.rvfi) rvW.wset(writeRd(d.rd, old.rdata, rvBase(pc + 4, False)));
    end
  endrule

  rule doCsrWrite (st == CsrWr);
    let _ <- csrf.regs.access(RegReq { addr: truncate(dec.csr), write: True,
                                       wdata: csrNv, wstrb: 4'hF });
    pc <= pc + 4;
    st <= Fetch;
    retire.send();
    if (cfg.rvfi) begin
      // rs2 这几位是 CSR 地址的一部分，不读寄存器，按规范给地址 0
      Rvfi r = readRs(dec.rs1, rvCsrRs1, 0, 0, rvBase(pc + 4, False));
      rvW.wset(writeRd(dec.rd, rvCsrOld, r));
    end
  endrule

  RegManager#(32, 32) iUp = interface RegManager;
      method Bool valid = needNow || wantPf;
      method RegReq#(32, 32) req = RegReq { addr: askAd, write: False,
                                            wdata: 0, wstrb: 4'hF };
      method Action ready(Bool v); iRdy._write(v); endmethod
      method Action resp(Bool v, RegRsp#(32) x);
        iRspV._write(v);
        iRspX._write(x);
      endmethod
    endinterface;
  RegManager#(32, 32) dUp = interface RegManager;
      method Bool valid = dValid;
      method RegReq#(32, 32) req = dReq;
      method Action ready(Bool v); dRdy._write(v); endmethod
      method Action resp(Bool v, RegRsp#(32) x);
        dRspV._write(v);
        dRspX._write(x);
      endmethod
    endinterface;

  // 关掉 mmu 就直通，一个门都不例化
  RegManager#(32, 32) iOut = iUp;
  RegManager#(32, 32) dOut = dUp;

  if (cfg.mmu) begin
    MmuIfc imu <- mkMmu(True);
    MmuIfc dmu <- mkMmu(False);
    mkPipe(iUp, imu.up);
    mkPipe(dUp, dmu.up);

    rule sfenceLatch;
      sfenceR <= sfenceP;
    endrule

    rule mmuCtl;
      Bit#(32) satp = {csrf.satp_mode, csrf.satp_asid, csrf.satp_ppn};
      // MPRV 只改读写的有效特权级，取指照旧（3.1.6.3）
      Bit#(2) dpriv = (csrf.mstatus_mprv == 1) ? csrf.mstatus_mpp : priv;
      Bool sum = csrf.mstatus_sum == 1;
      Bool mxr = csrf.mstatus_mxr == 1;
      imu.ctl(satp, priv, sum, mxr);
      dmu.ctl(satp, dpriv, sum, mxr);
      imu.fence(sfenceR);
      dmu.fence(sfenceR);
    endrule

    rule mmuFault;
      iPf <= imu.pageFault;
      dPf <= dmu.pageFault;
    endrule

    iOut = imu.down;
    dOut = dmu.down;
  end


  interface imem = iOut;
  interface dmem = dOut;

  interface HartIrq irq;
    method Action irq(Bool msip, Bool mtip, Bool meip, Bool ssipSet);
      ssipSetIn._write(ssipSet);
      msipIn._write(msip);
      mtipIn._write(mtip);
      meipIn._write(meip);
    endmethod
  endinterface

  interface HartPins pins;
    method Action hartid(Bit#(32) v); hidIn._write(v); endmethod
    method Action halt(Bool v); halted._write(v); endmethod
  endinterface

  interface rvfi = rvfiPins(rvR);
endmodule

endpackage
