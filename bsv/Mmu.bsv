package Mmu;

import Vector::*;
import ConfigReg::*;
import RegIf::*;
import Cam::*;

// Sv32 的地址翻译：一张 16 条全相联的 TLB 加一个两级页表遍历器。
//
// 条目数与相联度是 E29 量出来的：真 rv32 Linux 启动踪迹上 892 万次取指、
// 384 个虚页，16 条命中 99.89%，再翻一倍只多 0.05%；而相联度几乎不花钱
// ——TLB 是 CAM、只能用触发器，条目本身占了绝大部分（D88）。
//
// 形状取的是现成的两个契约：**上游是会停顿的目标**（翻译不了的时候把 ready
// 拉低），**下游是发起方**。遍历器不另开一个访存口，就借下游这一个——缺页
// 遍历的时候流水线本来就停着，没人跟它抢。
//
// 放在 hart 包里而不是单开一个仓：三级晋升那条判据要「有第二个消费者」才拆，
// 现在只有 hart 用它。

// TLB 里存的是**已经解析好的**物理页号，不是页表项本身。这样大页（一级叶子）
// 不必单独一路：遍历时就把低十位拼好，查表这一侧只有一种形状。
typedef struct {
  Bit#(20) ppn;
  Bit#(8)  flags;   // D A G U X W R V，位序同页表项的低八位
} Ent deriving (Bits, Eq, FShow);

interface MmuIfc;
  interface RegTarget#(32, 32)  up;     // 面向流水线
  interface RegManager#(32, 32) down;   // 面向存储
  (* always_ready, always_enabled *)
  method Action ctl(Bit#(32) satp, Bit#(2) priv, Bool sum, Bool mxr);
  (* always_ready, always_enabled *) method Action fence(Bool f);
  // 这一拍答复的那个错是缺页（而不是总线上的访问错）。两者的异常号不同，
  // 所以必须分得开——一位就够，因为一拍只答一笔。
  (* always_ready *) method Bool pageFault;
endinterface

module mkMmu#(Bool isFetch)(MmuIfc);
  Cam#(16, Bit#(20), Ent) tlb <- mkCam;

  Wire#(Bit#(32)) satpW  <- mkBypassWire;
  Wire#(Bit#(2))  privW  <- mkBypassWire;
  Wire#(Bool)     fenceW <- mkBypassWire;
  Wire#(Bool)     sumW   <- mkBypassWire;
  Wire#(Bool)     mxrW   <- mkBypassWire;

  Wire#(Bool)           upV <- mkBypassWire;
  Wire#(RegReq#(32, 32)) upR <- mkBypassWire;

  Wire#(Bool)        dnRdy  <- mkBypassWire;
  Wire#(Bool)        dnRspV <- mkBypassWire;
  Wire#(RegRsp#(32)) dnRspX <- mkBypassWire;

  // 0 空闲 · 1 一级页表项已经发出去 · 2 二级页表项已经发出去
  Reg#(Bit#(2))  wst  <- mkConfigReg(0);
  Reg#(Bit#(32)) wad  <- mkConfigReg(0);
  Reg#(Bit#(20)) wvpn <- mkConfigReg(0);

  // 翻译开着：satp.MODE 为一，且当前不是机器态。机器态不翻译（特权规范 4.1.11）。
  Bool on   = satpW[31] == 1 && privW != 2'b11;
  Bit#(20) vpn = upR.addr[31:12];
  Bit#(12) off = upR.addr[11:0];
  Maybe#(Ent) hit = tlb.lookup(vpn);

  // 权限：取指要 X，写要 W，读要 R，MXR 开着时可执行的页也读得出（3.1.6.3）。
  // U 页只给 U 态；SUM 开着时 S 态也能读写，但无论 SUM 如何都不许执行（4.3.1）。
  // A 没置上、或者写而 D 没置上，一律缺页（规范允许的两种做法之一，4.3.2）。
  function Bool permOk(Ent e);
    Bool kind = isFetch ? (e.flags[3] == 1)
              : (upR.write ? (e.flags[2] == 1)
                           : (e.flags[1] == 1 || (mxrW && e.flags[3] == 1)));
    Bool lvl  = (privW == 2'b00) ? (e.flags[4] == 1)
              : (e.flags[4] == 0 || (sumW && !isFetch));
    Bool acc  = e.flags[6] == 1 && (!upR.write || e.flags[7] == 1);
    return kind && lvl && acc;
  endfunction

  Bool  hitOk  = hit matches tagged Valid .e ? permOk(e) : False;
  Bool  hitBad = hit matches tagged Valid .e ? !permOk(e) : False;
  Bool  miss   = on && upV && !isValid(hit);

  Bit#(32) phys = case (hit) matches
                    tagged Valid .e: {e.ppn, off};
                    default: upR.addr;
                  endcase;

  // 页表项的合法性：V 要为一，而「可写不可读」是保留编码（规范 4.3.2）
  function Bool pteOk(Bit#(32) p) = p[0] == 1 && !(p[2] == 1 && p[1] == 0);
  function Bool isLeaf(Bit#(32) p) = p[1] == 1 || p[3] == 1;

  Reg#(Bool) faultR <- mkConfigReg(False);
  // 判定缺页的地方有两处（走表走到死路、命中但权限不对），而寄存器只许一条
  // 规则写——两条规则写同一个寄存器就是并行冲突。所以判定只发线。
  PulseWire  setF   <- mkPulseWire;
  // 走表时读页表项本身出错、或者翻出来的物理地址这颗芯片上根本没有，规范要的是
  // 访问错而不是缺页（4.3.2 第 2 步）。两者异常号不同，软件靠它分辨「页表写错了」
  // 与「物理内存不在」，所以要单记一位。
  Reg#(Bool) pfR    <- mkConfigReg(False);
  PulseWire  setA   <- mkPulseWire;

  // 清表时连正在走的那一趟也作废：它读到的页表项可能早于这次 sfence，
  // 学进去就是一条清不掉的旧翻译。请求还举着，下一拍重走。
  rule flushTlb (fenceW);
    tlb.flush;
    wst <= 0;
  endrule

  // 缺失就起步走表。根页表的物理地址是 satp.PPN 左移十二位，
  // 一级的下标是虚页号的高十位。答错的那一拍不起步：那一拍举着的还是
  // 刚判了缺页的那一笔，再走一趟会给下一笔请求凭空再答一个错。
  rule startWalk (miss && wst == 0 && !fenceW && !faultR);
    wad  <= {satpW[19:0], 12'b0} + {20'b0, vpn[19:10], 2'b0};
    wvpn <= vpn;
    wst  <= 1;
  endrule

  rule stepWalk (wst != 0 && dnRspV && !fenceW);
    Bit#(32) p = dnRspX.rdata;
    if (dnRspX.err) begin
      wst <= 0;
      setA.send();
    end else if (!pteOk(p)) begin
      wst <= 0;
      setF.send();
    end else if (isLeaf(p)) begin
      // 一级的叶子就是四兆大页：高位来自页表项，低十位照抄虚地址。
      // 大页的低十位必须为零，否则是「未对齐的大页」，按缺页处理（4.3.2）。
      //
      // Sv32 的物理地址有三十四位（PPN 二十二位），而我们的访存契约只有
      // 三十二位，所以只取低二十位。指到 4 GiB 以上的那块物理内存这颗芯片上
      // 不存在，与总线答错同一个待遇：访问错。
      Bit#(20) ppn = (wst == 1) ? {p[29:20], wvpn[9:0]} : p[29:10];
      if (p[31:30] != 0) setA.send();
      else if ((wst == 1) && (p[19:10] != 0)) setF.send();
      else tlb.learn(wvpn, Ent { ppn: ppn, flags: p[7:0] });
      wst <= 0;
    end else if (wst == 1) begin
      wad <= {p[29:10], 12'b0} + {20'b0, wvpn[9:0], 2'b0};
      wst <= 2;
    end else begin
      // 二级还不是叶子：没有第三级，这就是缺页
      wst <= 0;
      setF.send();
    end
  endrule

  // 权限不对的命中不必走表，直接判缺页。答错那一拍不判，理由同上。
  rule permFault (on && upV && hitBad && wst == 0 && !faultR);
    setF.send();
  endrule

  // 错只答一拍。原来等「上游把手放下」才清，而取指口从不放手：陷入之后
  // 它立刻举着新地址，标记于是一直挂着，此后每一笔取指都答错。
  // 这个错答给了哪一笔由 hart 自己记（fltAd），不归这里管。
  rule faultReg;
    faultR <= setF || setA;
    pfR    <= setF;
  endrule

  interface RegTarget up;
    method Action req(Bool v, RegReq#(32, 32) r);
      upV._write(v);
      upR._write(r);
    endmethod
    // 只许看寄存器。碰这一拍的请求（upV/upR）就是同一条规则又写又读同一根线；
    // 而碰下游的 ready 会绕出另一个环——`send` 一条规则里既写上游的请求、
    // 又读上游的 ready，若 ready 牵着下游的 ready，下游的 valid 又牵着上游的
    // 请求，一圈就闭合了（G0009，症状是 send 被整条丢掉）。
    //
    // 乐观地答「收得下」是安全的，契约写明了理由：发起方在收到答复之前把
    // valid 与 req 顶着不动，所以真正决定走不走的是 rspValid，不是 ready。
    method Bool ready = faultR ? True : (wst == 0);
    method Bool rspValid = faultR ? True : (wst == 0 && dnRspV);
    method RegRsp#(32) rsp = faultR ? RegRsp { rdata: 0, err: True } : dnRspX;
  endinterface

  interface RegManager down;
    method Bool valid = (wst != 0) ? True
                      : (upV && !faultR && (!on || hitOk));
    method RegReq#(32, 32) req =
      (wst != 0) ? RegReq { addr: wad, write: False, wdata: 0, wstrb: 4'hF }
                 : RegReq { addr: on ? phys : upR.addr, write: upR.write,
                            wdata: upR.wdata, wstrb: upR.wstrb };
    method Action ready(Bool v); dnRdy._write(v); endmethod
    method Action resp(Bool v, RegRsp#(32) x);
      dnRspV._write(v);
      dnRspX._write(x);
    endmethod
  endinterface

  method Action ctl(Bit#(32) satp, Bit#(2) priv, Bool sum, Bool mxr);
    satpW._write(satp);
    privW._write(priv);
    sumW._write(sum);
    mxrW._write(mxr);
  endmethod
  method Action fence(Bool f); fenceW._write(f); endmethod
  method Bool pageFault = faultR && pfR;
endmodule

endpackage
