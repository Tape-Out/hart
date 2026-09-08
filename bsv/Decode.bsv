package Decode;

// RV32I + Zicsr + M 的译码。纯组合，没有状态——译码器出错时看这一个函数就够。

typedef enum {
  OpAdd, OpSub, OpSll, OpSlt, OpSltu, OpXor, OpSrl, OpSra, OpOr, OpAnd,
  OpMul, OpMulh, OpMulhsu, OpMulhu, OpDiv, OpDivu, OpRem, OpRemu
} AluOp deriving (Bits, Eq, FShow);

typedef enum {
  Reg, Imm, Branch, Load, Store, Jal, Jalr, Lui, Auipc, Csr, Sys, Bad
} Kind deriving (Bits, Eq, FShow);

typedef enum { CsrRw, CsrRs, CsrRc } CsrOp deriving (Bits, Eq, FShow);

typedef struct {
  Kind      kind;
  AluOp     alu;
  Bit#(5)   rd;
  Bit#(5)   rs1;
  Bit#(5)   rs2;
  Bit#(32)  imm;
  Bit#(3)   fn3;
  Bit#(12)  csr;
  CsrOp     csrOp;
  Bool      csrImm;    // csrrwi 这一族，源是立即数不是寄存器
  Bool      isMul;
} Decoded deriving (Bits, FShow);

function Bit#(32) immI(Bit#(32) i) = signExtend(i[31:20]);
function Bit#(32) immS(Bit#(32) i) = signExtend({i[31:25], i[11:7]});
function Bit#(32) immB(Bit#(32) i) =
  signExtend({i[31], i[7], i[30:25], i[11:8], 1'b0});
function Bit#(32) immU(Bit#(32) i) = {i[31:12], 12'b0};
function Bit#(32) immJ(Bit#(32) i) =
  signExtend({i[31], i[19:12], i[20], i[30:21], 1'b0});

function Decoded decode(Bit#(32) i, Bool hasMul);
  Bit#(7) op  = i[6:0];
  Bit#(3) fn3 = i[14:12];
  Bit#(7) fn7 = i[31:25];

  Decoded d = Decoded {
    kind: Bad, alu: OpAdd, rd: i[11:7], rs1: i[19:15], rs2: i[24:20],
    imm: 0, fn3: fn3, csr: i[31:20], csrOp: CsrRw, csrImm: False, isMul: False };

  // M 扩展与基本运算共用 opcode 0110011，靠 funct7 分开
  AluOp base = case (fn3)
                 0: (fn7[5] == 1 ? OpSub : OpAdd);
                 1: OpSll;
                 2: OpSlt;
                 3: OpSltu;
                 4: OpXor;
                 5: (fn7[5] == 1 ? OpSra : OpSrl);
                 6: OpOr;
                 default: OpAnd;
               endcase;
  AluOp mop  = case (fn3)
                 0: OpMul;
                 1: OpMulh;
                 2: OpMulhsu;
                 3: OpMulhu;
                 4: OpDiv;
                 5: OpDivu;
                 6: OpRem;
                 default: OpRemu;
               endcase;

  case (op)
    7'b0110011: begin
      if (fn7 == 7'b0000001) begin
        d.kind  = hasMul ? Reg : Bad;
        d.alu   = mop;
        d.isMul = True;
      end else begin
        d.kind = Reg;
        d.alu  = base;
      end
    end
    7'b0010011: begin
      d.kind = Imm;
      d.imm  = immI(i);
      // 移位立即数的 funct7 也编码算术右移
      d.alu  = (fn3 == 1) ? OpSll
             : (fn3 == 5) ? (fn7[5] == 1 ? OpSra : OpSrl)
             : base;
    end
    7'b0000011: begin d.kind = Load;   d.imm = immI(i); end
    7'b0100011: begin d.kind = Store;  d.imm = immS(i); end
    7'b1100011: begin d.kind = Branch; d.imm = immB(i); end
    7'b1101111: begin d.kind = Jal;    d.imm = immJ(i); end
    7'b1100111: begin d.kind = Jalr;   d.imm = immI(i); end
    7'b0110111: begin d.kind = Lui;    d.imm = immU(i); end
    7'b0010111: begin d.kind = Auipc;  d.imm = immU(i); end
    7'b1110011: begin
      if (fn3 == 0) begin
        d.kind = Sys;              // ecall / ebreak / mret / wfi
        d.imm  = zeroExtend(i[31:20]);
      end else begin
        d.kind   = Csr;
        d.csrImm = (fn3[2] == 1);
        d.csrOp  = case (fn3[1:0]) 1: CsrRw; 2: CsrRs; default: CsrRc; endcase;
        d.imm    = zeroExtend(i[19:15]);   // csrrwi 一族的立即数就是 rs1 位段
      end
    end
    7'b0001111: d.kind = Sys;      // fence 当空操作，本实现顺序执行
  endcase
  return d;
endfunction

// 写不写寄存器堆。x0 永远不写，判一次省得每处都判。
function Bool writesReg(Decoded d) =
  d.rd != 0 && (d.kind == Reg || d.kind == Imm || d.kind == Load
                || d.kind == Jal || d.kind == Jalr || d.kind == Lui
                || d.kind == Auipc || d.kind == Csr);

endpackage
