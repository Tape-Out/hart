package Muldiv;

import Decode::*;

// M 扩展的执行单元。乘法四周期（E28 定案：单周期要 20,345 µm²，
// 比整颗 soc-mcu 还大；四周期只要 29.4%，而面积×拍数只差 18%）。
// 除法是恢复余数法，一位一拍。

interface MuldivIfc;
  (* always_ready *) method Bool     idle;
  (* always_ready *) method Action   start(AluOp op, Bit#(32) a, Bit#(32) b);
  (* always_ready *) method Bool     done;
  (* always_ready *) method Bit#(32) result;
endinterface

module mkMuldiv(MuldivIfc);
  Reg#(Bit#(6))  step  <- mkReg(0);      // 0 表示空闲
  Reg#(AluOp)    op    <- mkReg(OpMul);
  Reg#(Bit#(64)) acc   <- mkReg(0);
  Reg#(Bit#(64)) mcand <- mkReg(0);
  Reg#(Bit#(32)) mplier <- mkReg(0);
  Reg#(Bit#(32)) quot  <- mkReg(0);
  Reg#(Bool)     negQ  <- mkReg(False);
  Reg#(Bool)     negR  <- mkReg(False);
  Reg#(Bool)     fin   <- mkReg(False);

  Bool isMulOp = op == OpMul || op == OpMulh || op == OpMulhsu || op == OpMulhu;

  rule chew (step != 0 && isMulOp);
    // 每拍吃 8 位，四拍走完
    Bit#(64) part = mcand * zeroExtend(mplier[7:0]);
    Bit#(6)  sh   = 8 * (4 - zeroExtend(step));
    acc    <= acc + (part << sh);
    mplier <= mplier >> 8;
    step   <= step - 1;
    if (step == 1) fin <= True;
  endrule

  rule grind (step != 0 && !isMulOp);
    // 恢复余数：余数左移补一位被除数，够减就减并置商位
    Bit#(64) rem = {acc[62:0], zeroExtend(mplier[31])};
    Bit#(64) sub = rem - mcand;
    Bool fits = (rem >= mcand);
    acc    <= fits ? sub : rem;
    quot   <= {quot[30:0], fits ? 1'b1 : 1'b0};
    mplier <= mplier << 1;
    step   <= step - 1;
    if (step == 1) fin <= True;
  endrule

  method Bool idle = step == 0 && !fin;

  method Action start(AluOp o, Bit#(32) a, Bit#(32) b);
    op  <= o;
    fin <= False;
    acc <= 0;
    quot <= 0;
    case (o)
      OpMul, OpMulh, OpMulhsu, OpMulhu: begin
        Bool sa = (o == OpMul || o == OpMulh || o == OpMulhsu);
        Bool sb = (o == OpMul || o == OpMulh);
        mcand  <= sa ? signExtend(a) : zeroExtend(a);
        mplier <= b;
        step   <= 4;
      end
      default: begin
        Bool sg = (o == OpDiv || o == OpRem);
        Bit#(32) ua = (sg && a[31] == 1) ? (~a + 1) : a;
        Bit#(32) ub = (sg && b[31] == 1) ? (~b + 1) : b;
        mcand  <= zeroExtend(ub);
        mplier <= ua;
        negQ   <= sg && (a[31] != b[31]) && b != 0;
        negR   <= sg && a[31] == 1;
        step   <= 32;
      end
    endcase
  endmethod

  method Bool done = fin;

  method Bit#(32) result;
    case (op)
      OpMul:   return acc[31:0];
      OpMulh, OpMulhsu, OpMulhu: return acc[63:32];
      OpDiv, OpDivu: return negQ ? (~quot + 1) : quot;
      default: begin
        Bit#(32) rem = acc[31:0];
        return negR ? (~rem + 1) : rem;
      end
    endcase
  endmethod
endmodule

// 关掉 M 扩展时用这个。光靠译码不出乘法指令是不够的——模块还在那儿，
// 综合器留着它的复位逻辑与输出，实测面积几乎没变（43,192 对 43,289）。
// 特性开关要真的不例化，才叫省下来了。
module mkMuldivNone(MuldivIfc);
  method Bool     idle   = True;
  method Action   start(AluOp op, Bit#(32) a, Bit#(32) b);
    noAction;
  endmethod
  method Bool     done   = False;
  method Bit#(32) result = 0;
endmodule

endpackage
