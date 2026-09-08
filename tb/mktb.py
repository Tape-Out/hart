"""生成 hart 的测试台：一段真程序、一块存储、一条自检。

判据不是「跑起来了」而是「结果对不对」：程序把每个算式的结果写到 0x1000_0000，
测试台按顺序对期望值，错一个就打出来并置退出码。
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from rasm import assemble  # noqa: E402

MAGIC = 0x1000_0000

# 每条 sw 写出一个期望值，顺序即下面 EXPECT 的顺序
SRC = [
    "  lui  a0, 0x10000",        # a0 = 0x1000_0000，自检口
    "  addi t0, zero, 5",
    "  addi t1, zero, 7",
    "  add  t2, t0, t1",         # 12
    "  sw   t2, 0(a0)",
    "  sub  t2, t1, t0",         # 2
    "  sw   t2, 0(a0)",
    "  slli t2, t0, 2",          # 20
    "  sw   t2, 0(a0)",
    "  xor  t2, t0, t1",         # 5 ^ 7 = 2
    "  sw   t2, 0(a0)",
    "  slt  t2, t0, t1",         # 1
    "  sw   t2, 0(a0)",
    "  mul  t2, t0, t1",         # 35
    "  sw   t2, 0(a0)",
    "  addi t3, zero, -20",
    "  div  t2, t3, t0",         # -20 / 5 = -4
    "  sw   t2, 0(a0)",
    "  rem  t2, t1, t0",         # 7 % 5 = 2
    "  sw   t2, 0(a0)",
    # 分支与跳转
    "  addi t2, zero, 0",
    "  beq  t0, t1, skip",       # 不跳
    "  addi t2, t2, 1",
    "skip:",
    "  bne  t0, t1, taken",      # 跳
    "  addi t2, t2, 100",        # 跳过
    "taken:",
    "  sw   t2, 0(a0)",          # 1
    # 访存：写进 RAM 再读回来，顺带验字节与半字
    "  lui  a1, 0x80010",
    "  addi t2, zero, 0x7B",
    "  sw   t2, 0(a1)",
    "  lw   t2, 0(a1)",
    "  sw   t2, 0(a0)",          # 0x7B
    "  addi t2, zero, -1",
    "  sb   t2, 4(a1)",
    "  lbu  t2, 4(a1)",
    "  sw   t2, 0(a0)",          # 0xFF
    "  lb   t2, 4(a1)",
    "  sw   t2, 0(a0)",          # -1
    # CSR：读写 mscratch，再用 set 位
    "  addi t2, zero, 0x55",
    "  csrrw zero, 0x340, t2",
    "  csrrs t2, 0x340, zero",
    "  sw   t2, 0(a0)",          # 0x55
    "  addi t3, zero, 0x2A",
    "  csrrs zero, 0x340, t3",
    "  csrrs t2, 0x340, zero",
    "  sw   t2, 0(a0)",          # 0x7F
    "done:",
    "  jal  zero, done",
]
EXPECT = [12, 2, 20, 2, 1, 35, (-4) & 0xFFFFFFFF, 2, 1,
          0x7B, 0xFF, 0xFFFFFFFF, 0x55, 0x7F]

prog = assemble(SRC)
out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)

rom = "\n".join(f"      {i}: return 32'h{w:08X};" for i, w in enumerate(prog))
exp = "\n".join(f"      {i}: return 32'h{v:08X};" for i, v in enumerate(EXPECT))

(out / "Prog.bsv").write_text(f"""package Prog;

// 由 tb/mktb.py 生成，勿手改。改程序改那个脚本。

Integer progLen = {len(prog)};
Integer expLen  = {len(EXPECT)};

function Bit#(32) romWord(Bit#(32) i);
  case (i)
{rom}
    default: return 32'h00000013;   // nop
  endcase
endfunction

function Bit#(32) expected(Bit#(32) i);
  case (i)
{exp}
    default: return 32'hDEADBEEF;
  endcase
endfunction

endpackage
""", encoding="utf-8")

(out / "Tb.bsv").write_text('''package Tb;

import RegFile::*;
import RegIf::*;
import Hart::*;
import Prog::*;

// 核的自检台。判据不是「跑起来了」而是「结果对不对」：程序把每个算式的结果
// 写到 0x1000_0000，这里按顺序对期望值。

// 出问题时把它改成 True，每次访存都打出来。上一次逮到的就是这么逮到的：
// addi rd, rs, -1 被译成 sub，因为立即数型借用了 funct7。
Bool trace = False;

(* synthesize *)
module mkTb(Empty);
  HartIfc#(12, 32) cpu <- mkHart(HartCfg { mul: True, smode: False });
  RegFile#(Bit#(8), Bit#(32)) ram <- mkRegFileFull;

  Reg#(Bit#(32)) cyc  <- mkReg(0);
  Reg#(Bit#(32)) seen <- mkReg(0);
  Reg#(Bool)     bad  <- mkReg(False);

  function Bool inRom(Bit#(32) a) = a[31:28] == 4'h8 && a[16] == 0;
  function Bool inRam(Bit#(32) a) = a[31:28] == 4'h8 && a[16] == 1;
  function Bool isOut(Bit#(32) a) = a[31:28] == 4'h1;

  rule tick;
    cyc <= cyc + 1;
    if (cyc > 20000) begin
      $display("TIMEOUT after %0d cycles", cyc);
      $finish(1);
    end
  endrule

  // 取指口：ROM 组合读出
  rule fetch;
    Bit#(32) a = cpu.imem.req.addr;
    Bit#(32) w = inRom(a) ? romWord((a - 32'h8000_0000) >> 2) : 32'h00000013;
    cpu.imem.ready(cpu.imem.valid);
    cpu.imem.resp(cpu.imem.valid, RegRsp { rdata: w, err: False });
  endrule

  // 访存口：RAM 与自检口
  rule dmem;
    let r = cpu.dmem.req;
    Bit#(32) rd = 0;
    if (cpu.dmem.valid) begin
      if (inRam(r.addr)) begin
        Bit#(8) i = truncate(r.addr >> 2);
        Bit#(32) old = ram.sub(i);
        rd = old;
        if (r.write) ram.upd(i, applyStrb(old, r.wdata, r.wstrb));
        if (trace)
          $display("MEM %s a=%08h i=%0d strb=%b wd=%08h old=%08h",
                   r.write ? "W" : "R", r.addr, i, r.wstrb, r.wdata, old);
      end else if (isOut(r.addr) && r.write) begin
        Bit#(32) want = expected(seen);
        if (r.wdata != want) begin
          $display("FAIL check %0d: got %08h want %08h", seen, r.wdata, want);
          bad <= True;
        end
        seen <= seen + 1;
        if (seen + 1 == fromInteger(expLen)) begin
          if (bad) $display("FAILED");
          else $display("PASS all %0d checks", expLen);
          $finish(bad ? 1 : 0);
        end
      end
    end
    cpu.dmem.ready(cpu.dmem.valid);
    cpu.dmem.resp(cpu.dmem.valid, RegRsp { rdata: rd, err: False });
  endrule

  rule plat;
    cpu.pins.irq(False, False, False);
    cpu.pins.hartid(0);
    cpu.pins.halt(False);
  endrule
endmodule

endpackage
''', encoding="utf-8")
print(f"  程序 {len(prog)} 条指令，自检 {len(EXPECT)} 项")
