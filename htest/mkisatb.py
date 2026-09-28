"""riscv-tests 的 rv32ui 与 rv32um（物理环境 p），一个测试一个测试台。

判据照上游：跑完往 tohost 写 1 是过，写别的数是第 (值 >> 1) 项没过。tohost 的地址
逐个从 ELF 里读，不写死——多数在 0x8000_1000，代码长的会被推到下一页。

只在 mmu、rvfi、imsic 都关着的点上跑：p 环境不开翻译，那三样换了也测不到别的。
没有工具链就报错退出，不悄悄跳过。
"""
import json
import os
import pathlib
import shutil
import subprocess
import sys

here = pathlib.Path(__file__).resolve().parent
up = here.parent / "third_party/riscv-tests"
out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
k = cfg.get("knobs", {})
mul = bool(k.get("mul", True))
smode = bool(k.get("smode", False))
if any(k.get(x) for x in ("mmu", "rvfi", "imsic")):
    sys.exit(0)

pre = os.environ.get("RISCV_PREFIX") or next(
    (p for p in ("riscv64-unknown-elf-", "riscv64-linux-gnu-") if shutil.which(p + "gcc")), None)
if pre is None:
    sys.exit("没有 RISC-V 工具链：装 gcc-riscv64-unknown-elf，或用 RISCV_PREFIX 指一个")
if not (up / "env/p/link.ld").is_file():
    sys.exit(f"{up} 不全：git submodule update --init --recursive")

WORDS = 1 << 14
# 不适用的：非对齐访存这颗核是报异常的（规范允许，自检台查过），ma_data 要硬件支持或 M 态模拟
SKIP = {"rv32ui-p-ma_data"}
build = out / f"isa{label}"
build.mkdir(parents=True, exist_ok=True)
suites = ["rv32ui"] + (["rv32um"] if mul else [])
march = ("rv32im" if mul else "rv32i") + "_zicsr_zifencei"
n = 0
for suite in suites:
    for s in sorted((up / "isa" / suite).glob("*.S")):
        name = f"{suite}-p-{s.stem}"
        if name in SKIP:
            continue
        elf = build / f"{name}.elf"
        subprocess.run([pre + "gcc", f"-march={march}", "-mabi=ilp32", "-static", "-mcmodel=medany",
                        "-fvisibility=hidden", "-nostdlib", "-nostartfiles",
                        f"-I{up}/env/p", f"-I{up}/isa/macros/scalar", f"-T{up}/env/p/link.ld",
                        str(s), "-o", str(elf)], check=True)
        syms = subprocess.run([pre + "nm", str(elf)], check=True, capture_output=True,
                              text=True).stdout.split("\n")
        tohost = next(int(x.split()[0], 16) for x in syms if x.endswith(" tohost"))
        raw = build / f"{name}.bin"
        subprocess.run([pre + "objcopy", "-O", "binary", str(elf), str(raw)], check=True)
        b = raw.read_bytes()
        if len(b) > 4 * WORDS:
            sys.exit(f"{name} 有 {len(b)} 字节，放不进 {4 * WORDS} 字节的存储")
        b += bytes(-len(b) % 4)
        words = [int.from_bytes(b[i:i + 4], "little") for i in range(0, len(b), 4)]
        hexf = build / f"{name}.hex"
        hexf.write_text("\n".join(f"{w:08x}" for w in words + [0] * (WORDS - len(words))) + "\n")
        cap = "Isa" + "".join(x.capitalize() for x in (suite + "_" + s.stem).split("_")) + label
        (out / f"{cap}Tb.bsv").write_text(f'''package {cap}Tb;

import RegFile::*;
import RegIf::*;
import Hart::*;
import RvfiPins::*;

(* synthesize *)
module mk{cap}Tb(Empty);
  HartIfc#(12, 32) cpu <- mkHart(HartCfg {{ mul: {mul}, smode: {smode}, mmu: False,
                                            rvfi: False, imsic: False }});
  RegFile#(Bit#(14), Bit#(32)) mem <- mkRegFileLoad("{hexf}", 0, {WORDS - 1});
  Reg#(Bit#(32)) cyc <- mkReg(0);

  function Bool inMem(Bit#(32) a) = a[31:16] == 16'h8000;
  function Bit#(14) ix(Bit#(32) a) = a[15:2];
  function Bit#(32) merge(Bit#(32) old, Bit#(32) d, Bit#(4) s);
    Bit#(32) m = {{s[3] == 1 ? 8'hFF : 0, s[2] == 1 ? 8'hFF : 0, s[1] == 1 ? 8'hFF : 0, s[0] == 1 ? 8'hFF : 0}};
    return (old & ~m) | (d & m);
  endfunction

  rule tick;
    cyc <= cyc + 1;
    if (cyc > 200000) begin
      $display("TIMEOUT {name}");
      $finish(1);
    end
  endrule

  rule fetch;
    Bit#(32) a = cpu.imem.req.addr;
    cpu.imem.ready(cpu.imem.valid);
    cpu.imem.resp(cpu.imem.valid, RegRsp {{ rdata: inMem(a) ? mem.sub(ix(a)) : 0, err: !inMem(a) }});
  endrule

  rule dmem;
    let r = cpu.dmem.req;
    Bit#(32) rd = 0;
    if (cpu.dmem.valid && inMem(r.addr)) begin
      Bit#(32) old = mem.sub(ix(r.addr));
      rd = old;
      if (r.write) begin
        mem.upd(ix(r.addr), merge(old, r.wdata, r.wstrb));
        if (r.addr == 32'h{tohost:08x}) begin
          if (r.wdata == 1) begin
            $display("PASS {name} in %0d cycles", cyc);
            $finish(0);
          end else begin
            $display("FAIL {name}: case %0d", r.wdata >> 1);
            $finish(1);
          end
        end
      end
    end
    cpu.dmem.ready(cpu.dmem.valid);
    cpu.dmem.resp(cpu.dmem.valid, RegRsp {{ rdata: rd, err: cpu.dmem.valid && !inMem(r.addr) }});
  endrule

  rule plat;
    cpu.irq.irq(False, False, False, False);
    cpu.pins.hartid(0);
    cpu.pins.halt(False);
  endrule
endmodule

endpackage
''', encoding="utf-8")
        n += 1
print(f"  riscv-tests {n} 个（{' '.join(suites)}，mul={mul} smode={smode}）")
