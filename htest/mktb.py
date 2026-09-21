"""生成 hart 的测试台：一段真程序、一块存储、一条自检。

判据不是「跑起来了」而是「结果对不对」：程序把每个算式的结果写到 0x1000_0000，
测试台按顺序对期望值，错一个就打出来并置退出码。

认矩阵：`mul` 关掉时那三条 M 指令是非法指令，整段连同期望值一起去掉——
不去掉的话测的是「核怎么处理非法指令」，不是「核算得对不对」，两件事。
`smode` 的那一段两个方向都跑：开着时 mstatus.sie 写得进去，关着时读回必须是零。
门控写漏了的表现是「关掉了硬件还在」，只有后一半看得出来。
陷入段不靠特权级那一段，所有配置都跑：`minstret` 不算陷入的指令，访存地址与跳转目标不对齐各报各的异常，
`mepc`/`sepc` 低两位读回为零。
`rvfi` 开时另查提交记录：编号连续，前后两条的 pc 接得上，写自检口的记录与期望值对得上。
`mmu` 开时在 S 态那一段里接着开翻译，验五件事：真的翻译了（虚实两个地址读写互通）·
TLB 真的缓存了 · sfence.vma 真的清了 · 三种权限各拒一次 · 取指缺页回得来。
"""
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from rasm import assemble  # noqa: E402

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
k = cfg.get("knobs", {})
mul = bool(k.get("mul", True))
smode = bool(k.get("smode", False))
mmu = bool(k.get("mmu", False))
rvfi = bool(k.get("rvfi", False))
imsic = bool(k.get("imsic", False))


def li(r, v):
    lo = v & 0xFFF
    lo = lo - 0x1000 if lo & 0x800 else lo
    return [f"  lui  {r}, {((v - lo) >> 12) & 0xFFFFF:#x}", f"  addi {r}, {r}, {lo}"]

HEAD = [
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
]
HEAD_EXP = [12, 2, 20, 2, 1]

MEXT = [
    "  mul  t2, t0, t1",         # 35
    "  sw   t2, 0(a0)",
    "  addi t3, zero, -20",
    "  div  t2, t3, t0",         # -20 / 5 = -4
    "  sw   t2, 0(a0)",
    "  rem  t2, t1, t0",         # 7 % 5 = 2
    "  sw   t2, 0(a0)",
]
MEXT_EXP = [35, (-4) & 0xFFFFFFFF, 2]

TAIL = [
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
]
TAIL_EXP = [1, 0x7B, 0xFF, 0xFFFFFFFF, 0x55, 0x7F]

# S 级的三个 CSR 是 M 级那三个的**掩码视图**（regmap 的 alias），不是第二份状态。
# 所以这一段从 S 级的 sstatus 写进去，再从 M 级的 mstatus 读回来——
# 读得到才说明两个地址共用同一份存储；给别名单独开存储的话这里就是 0。
# 两个方向都跑：smode 关掉时 sstatus 不存在，写了也读不出来。
SMODE = [
    "  addi t2, zero, 2",           # sie 是第 1 位
    "  csrrs zero, 0x100, t2",      # 从 S 级视图写
    "  csrrs t2, 0x300, zero",      # 从 M 级读回来
    "  andi t2, t2, 2",
    "  sw   t2, 0(a0)",            # smode 开 -> 2，关 -> 0
]
# 特权级这一段只在 smode 开时有意义：关掉时 sret 与 S 级 CSR 都不存在，
# 跑它测的是「核怎么处理非法指令」，不是委托对不对。
#
# 五件事一次走完：委托生效（S 态 ecall 直接进 stvec，不经 M）、sret 回得来、
# S 态够不着 M 级 CSR（非法指令且**没**被委托，所以落在 M 手里）、
# 一次软件写起来的 S 软件中断（写 sip.SSIP）、
# 以及一次**硬件送来的**——SSWI 写 SETSSIP 只送一个边沿，
# 由 mip.SSIP 那一位把它记住。没有这一条，`ssip_set` 那根线接没接上看不出来。
# 测试程序进 M 态的门是 ebreak（没委托），第几次记在 mscratch。
# 第一次：置 MPRV；开着翻译时读两次，MPP 为 S 按 S 翻译、MPP 为 M 不翻译；再开 TVM、TW、TSR。
# MPRV 故意留着，让 mret 回 S 时去清。第二次：读回 MPRV 必须是零，三位关回去。
MBRK = [
    "mbrk:",
    "  csrrs t2, 0x340, zero",
    "  bne  t2, zero, mbrk2",
    "  addi t2, zero, 1",
    "  csrrw zero, 0x340, t2",
    *li("t2", 0x2_0000), "  csrrs zero, 0x300, t2",
    *([*li("a5", 0x4000_0040), "  lw   t2, 0(a5)", "  sw   t2, 0(a0)",
       *li("t2", 0x1800), "  csrrs zero, 0x300, t2",
       "  lw   t2, 0(a5)", "  sw   t2, 0(a0)",
       *li("t2", 0x1000), "  csrrc zero, 0x300, t2"] if mmu else []),
    *li("t2", 0x70_0000), "  csrrs zero, 0x300, t2",
    "  jal  zero, mskip",
    "mbrk2:",
    "  csrrs t2, 0x300, zero",
    "  srli t2, t2, 17",
    "  andi t2, t2, 1",
    "  sw   t2, 0(a0)",
    *li("t2", 0x70_0000), "  csrrc zero, 0x300, t2",
    "  jal  zero, mskip",
]

DELEG = [
  "  lui  t2, hi(mhand)",
  "  addi t2, t2, lo(mhand)",
  "  csrrw zero, 0x305, t2",       # mtvec
  "  lui  t2, hi(shand)",
  "  addi t2, t2, lo(shand)",
  "  csrrw zero, 0x105, t2",       # stvec
  "  addi t2, zero, 0x300",
  "  csrrw zero, 0x302, t2",       # medeleg：委托 ecall-from-U(8) 与 -from-S(9)
  "  addi t2, zero, 2",
  "  csrrw zero, 0x303, t2",       # mideleg：委托 S 软件中断
  "  addi t2, zero, 1",
  "  slli t2, t2, 11",
  "  csrrw zero, 0x300, t2",       # mstatus.mpp = S
  "  lui  t2, hi(sgo)",
  "  addi t2, t2, lo(sgo)",
  "  csrrw zero, 0x341, t2",       # mepc = sgo
  "  mret",                        # 落到 S 态
"sgo:",
  "  ecall",                       # 委托了 -> 直接进 shand
  "  addi t2, zero, 55",
  "  sw   t2, 0(a0)",
  "  csrrs t2, 0xF14, zero",       # S 态读 mhartid -> 非法指令 -> mhand
  "  addi t2, zero, 66",
  "  sw   t2, 0(a0)",
  "  addi t2, zero, 2",
  "  csrrs zero, 0x104, t2",       # sie.ssie
  "  addi t2, zero, 2",
  "  csrrs zero, 0x100, t2",       # sstatus.sie
  "  addi t2, zero, 2",
  "  csrrs zero, 0x144, t2",       # sip.ssip -> 中断
  "  addi t2, zero, 77",
  "  sw   t2, 0(a0)",
  "  jal  zero, sdone",
"shand:",
  "  csrrs t2, 0x142, zero",       # scause
  "  sw   t2, 0(a0)",
  "  blt  t2, zero, sirq",         # 最高位是 1 就是中断
  "  csrrs t2, 0x141, zero",
  "  addi t2, t2, 4",
  "  csrrw zero, 0x141, t2",       # 异常要跳过闯祸那条
  "  sret",
"sirq:",
  "  addi t2, zero, 2",
  "  csrrc zero, 0x144, t2",       # 清掉 sip.ssip，否则回去就再来一次
  "  sret",
"mhand:",
  "  csrrs t2, 0x342, zero",       # mcause
  "  addi t3, zero, 3",
  "  beq  t2, t3, mbrk",           # ebreak 是进 M 态的门，不记号
  "  addi t2, t2, 0x100",          # 打个记号。不打的话委托没生效也看不出来——
  "  sw   t2, 0(a0)",              # M 手里的 mcause 与 S 手里的 scause 同是 9
"mskip:",
  "  csrrs t2, 0x341, zero",
  "  addi t2, t2, 4",
  "  csrrw zero, 0x341, t2",
  "  mret",
  *MBRK,
"sdone:",
  "  addi t2, zero, 88",
  "  sw   t2, 0(a0)",             # 软件那条走完了
"swait:",
  "  jal  zero, swait",           # 卡在这儿等硬件那条：测试台送一拍 ssip_set
]
# 末尾那个 0x80000001 是第二次进 shand 存下的 scause：只有边沿真的到了才会有
DELEG_EXP = [9, 55, 0x102, 66, 0x80000001, 77, 88, 0x80000001]

# 二级页表由程序自己写进 RAM（0x8001_3000），根页表是测试台里的常量。
# 虚页 0x40000..0x40006 的七项：可读写 · 只读 · 无效 · 用户页 · 指到 4 GiB 以上 ·
# 可写而 D 没置上 · A 没置上。只读页把 D 置上：原来没置，写它先被 D 那一道拦住，
# 把 W 那一道删掉照样全绿（变异实测）。每一道拦截要有一项只有它拦得住。
# 虚页 0x40400 走根页表 0x101 项，它指向的二级页表落在测试台答错的地址上。
#
# 规范允许「改了页表项而没 sfence」时新旧翻译任取其一。这里要求读到旧的：
# 验的是 TLB 真的缓存了。每次都走表的实现照样合规，但那就不是 E29 定的设计。
MMU = [
    *li("a1", 0x8001_3000),
    *li("t2", 0x2000_40C7), "  sw   t2, 0(a1)",
    *li("t2", 0x2000_40C3), "  sw   t2, 4(a1)",
    "  sw   zero, 8(a1)",
    *li("t2", 0x2000_4053), "  sw   t2, 12(a1)",
    *li("t2", 0xC000_4043), "  sw   t2, 16(a1)",
    *li("t2", 0x2000_4047), "  sw   t2, 20(a1)",
    *li("t2", 0x2000_4083), "  sw   t2, 24(a1)",
    *li("t2", 0x2000_405B), "  sw   t2, 28(a1)",   # 7：U X R，SUM 开着也不许 S 态执行
    *li("t2", 0x2000_4049), "  sw   t2, 32(a1)",   # 8：只可执行，MXR 开着才读得出
    *li("t2", 0x8008_0020), "  csrrw zero, 0x180, t2",   # satp：Sv32，根在 0x8002_0000
    "  sfence.vma",
    *li("a2", 0x8001_0040), *li("a3", 0x4000_0040), *li("a4", 0x8001_2040),
    *li("t2", 0x1111_1111), "  sw   t2, 0(a2)",
    "  lw   t2, 0(a3)", "  sw   t2, 0(a0)",
    *li("t2", 0x2222_2222), "  sw   t2, 0(a3)",
    "  lw   t2, 0(a2)", "  sw   t2, 0(a0)",
    *li("t2", 0x3333_3333), "  sw   t2, 0(a4)",
    *li("t2", 0x2000_48C7), "  sw   t2, 0(a1)",           # 0 号改指物理页 0x80012
    "  lw   t2, 0(a3)", "  sw   t2, 0(a0)",
    "  sfence.vma",
    "  lw   t2, 0(a3)", "  sw   t2, 0(a0)",
    *li("a5", 0x4000_1040),
    "  lw   t2, 0(a5)", "  sw   t2, 0(a0)",
    "  sw   t2, 0(a5)",
    *li("a5", 0x4000_2040), "  lw   t2, 0(a5)",
    *li("a5", 0x4000_3040), "  lw   t2, 0(a5)",
    *li("a5", 0x4000_5040), "  sw   t2, 0(a5)",
    *li("a5", 0x4000_6040), "  lw   t2, 0(a5)",
    *li("a5", 0x4000_1000), "  jalr ra, 0(a5)",
    *li("a5", 0x4000_4040), "  lw   t2, 0(a5)",
    *li("a5", 0x4040_0040), "  lw   t2, 0(a5)",
    *li("t2", 0x40000), "  csrrs zero, 0x100, t2",
    *li("a5", 0x4000_3040), "  lw   t2, 0(a5)", "  sw   t2, 0(a0)",
    *li("a5", 0x4000_7000), "  jalr ra, 0(a5)",
    *li("t2", 0x40000), "  csrrc zero, 0x100, t2",
    *li("a5", 0x4000_8040), "  lw   t2, 0(a5)",
    *li("t2", 0x80000), "  csrrs zero, 0x100, t2",
    "  lw   t2, 0(a5)", "  sw   t2, 0(a0)",
    *li("t2", 0x80000), "  csrrc zero, 0x100, t2",
]
MMU_EXP = [0x11111111, 0x22222222,       # 经虚地址读到实地址写的，反过来也一样
           0x22222222, 0x33333333,       # sfence 前是旧翻译，之后是新的
           0x22222222,                   # 只读页读得出来
           15, 0x40001040,               # 只读页写：写缺页
           13, 0x40002040,               # 无效页读：读缺页
           13, 0x40003040,               # S 态碰用户页：读缺页
           # A/D 没置上一律判缺页，不由硬件替软件置位（规范允许的两种做法之一）
           15, 0x40005040,
           13, 0x40006040,
           12, 0x40001000,               # 跳进不可执行页：取指缺页
           # 物理地址超出 32 位、走表时读页表项本身出错：规范要的是访问错（5），
           # 不是缺页。访问错没委托，落在 M 手里记 0x105
           0x105, 0x105,
           # SUM 开着 S 态读得出 U 页、仍不许执行；只可执行的页 MXR 开着才读得出
           0x22222222, 12, 0x40007000, 13, 0x40008040, 0x22222222]

# 接在 77 之后、sdone 之前：最后一项要留给硬件 SSWI 的边沿，测试台按检查计数送它
if mmu:
    i = DELEG.index("  addi t2, zero, 0x300")
    DELEG[i:i + 1] = li("t2", 0xB300)          # 另委托三种缺页 12/13/15
    i = DELEG.index("  blt  t2, zero, sirq") + 1
    DELEG[i:i] = [
        "  addi t3, zero, 12",
        "  blt  t2, t3, sskip",
        "  csrrs t3, 0x143, zero",
        "  sw   t3, 0(a0)",
        "  addi t3, zero, 12",
        "  bne  t2, t3, sskip",
        # 取指缺页的 sepc 就是坏地址，加 4 仍在坏页上；回跳进去之前存下的 ra
        "  csrrw zero, 0x141, ra",
        "  sret",
        "sskip:",
    ]
    i = DELEG.index("  jal  zero, sdone")
    DELEG[i:i] = MMU
    j = DELEG_EXP.index(77) + 1
    DELEG_EXP[j:j] = MMU_EXP

# 够不着的特权指令必须非法（特权规范 3.3.2、4.2.1）：M 以下的 mret、U 态的
# sret 与 sfence.vma。非法指令没委托，落在 M 手里记 0x102。
# 去 U 那一趟之后就留在 U 里把尾巴走完：S 软件中断从 U 态照样收得到，
# 低于 S 的特权级里 S 中断总是开着的。开着翻译时先关掉，代码页没有 U 位。
PRIV = [
    "  mret",
    *(["  csrrw zero, 0x180, zero", "  sfence.vma"] if mmu else []),
    "  addi t2, zero, 0x100",
    "  csrrc zero, 0x100, t2",
    "  lui  t2, hi(ugo)",
    "  addi t2, t2, lo(ugo)",
    "  csrrw zero, 0x141, t2",
    "  sret",
    "ugo:",
    "  sret",
    "  sfence.vma",
    "  mret",
]
# 两次 ebreak 之间四条，S 态执行都得是非法指令：TVM 下碰 satp、执行 sfence.vma，
# TSR 下执行 sret，TW 下执行 wfi
MACH = ["  ebreak", "  csrrs t2, 0x180, zero", "  sfence.vma", "  sret", "  wfi", "  ebreak"]
MACH_EXP = ([0x33333333, 0] if mmu else []) + [0x102] * 4 + [0]
i = DELEG.index("  mret")
DELEG[i:i] = ["  csrrw zero, 0x340, zero"]
i = DELEG.index("  jal  zero, sdone")
DELEG[i:i] = MACH + PRIV
j = DELEG_EXP.index(88)
DELEG_EXP[j:j] = MACH_EXP + [0x102] * 4

# 陷入段。mtvec 指到陷入那条的下一条，陷入之后顺着往下走，不必写处理程序。
# 一，minstret 不算陷入的指令（特权规范 3.3.1）：两次读之间退休的只有第一次读，ecall 不算，差 1。
#     执行拍退休的普通指令照计：读、四条 addi、读，差 5。上一条的 1 走的是 CSR 规则，只靠它查不出执行拍漏计。
# 二，访存口一笔只碰一个字，地址不对齐就陷入：整字读低两位非零报 4，半字写奇地址报 6，tval 是地址。
# 三，跳转目标不对齐（没有 C 扩展，IALIGN 是 32）在跳转这一条上报 0，tval 是目标。
# 四，mepc、sepc 低两位只读零（3.1.14、4.1.7）。
# a1 是 TAIL 里设好的 RAM 基址 0x8001_0000
def vec(label):
    return [f"  lui  t2, hi({label})", f"  addi t2, t2, lo({label})", "  csrrw zero, 0x305, t2"]


TRAPS = [
    *vec("trap1"),
    "  csrrs t3, 0xB02, zero",
    "  ecall",
"trap1:",
    "  csrrs t4, 0xB02, zero",
    "  sub  t2, t4, t3",
    "  sw   t2, 0(a0)",              # 1
    "  csrrs t3, 0xB02, zero",
    "  addi t2, zero, 1",
    "  addi t2, zero, 2",
    "  addi t2, zero, 3",
    "  addi t2, zero, 4",
    "  csrrs t4, 0xB02, zero",
    "  sub  t2, t4, t3",
    "  sw   t2, 0(a0)",              # 5
    *vec("trap2"),
    "  lw   t2, 1(a1)",
"trap2:",
    "  csrrs t2, 0x342, zero",
    "  sw   t2, 0(a0)",              # 4
    "  csrrs t2, 0x343, zero",
    "  sub  t2, t2, a1",
    "  sw   t2, 0(a0)",              # 1
    *vec("trap3"),
    "  sh   t2, 3(a1)",
"trap3:",
    "  csrrs t2, 0x342, zero",
    "  sw   t2, 0(a0)",              # 6
    "  csrrs t2, 0x343, zero",
    "  sub  t2, t2, a1",
    "  sw   t2, 0(a0)",              # 3
    *vec("trap4"),
    "  jalr ra, 2(t2)",
"trap4:",
    "  csrrs t3, 0x342, zero",
    "  sw   t3, 0(a0)",              # 0
    "  csrrs t3, 0x343, zero",
    "  sub  t3, t3, t2",
    "  sw   t3, 0(a0)",              # 2
    "  addi t2, zero, -1",
    "  csrrw zero, 0x341, t2",
    "  csrrs t3, 0x341, zero",
    "  sw   t3, 0(a0)",              # 0xFFFFFFFC
    *(["  csrrw zero, 0x141, t2",
       "  csrrs t3, 0x141, zero",
       "  sw   t3, 0(a0)"] if smode else []),
]
TRAPS_EXP = [1, 5, 4, 1, 6, 3, 0, 2, 0xFFFFFFFC] + ([0xFFFFFFFC] if smode else [])

# 间接 CSR 接中断文件（AIA 2.1、3.7–3.9）：miselect 选址，mireg 是窗口，mtopei 读最高号、写即领取。
# 测试台里的假中断文件：5 号挂起，阈值放行。处理程序跳过出错那条并报哨兵，所以实现没做时红而不挂。
IMSIC = [
    *vec("imtrap"),
    "  addi t2, zero, 0x70",          # miselect = eidelivery
    "  csrrw zero, 0x350, t2",
    "  csrrs t3, 0x350, zero",
    "  sw   t3, 0(a0)",               # 0x70：选址寄存器存得住 8 位
    "  addi t2, zero, 1",
    "  csrrw zero, 0x351, t2",        # 经窗口开投递
    "  csrrs t3, 0x351, zero",
    "  sw   t3, 0(a0)",               # 1：窗口读回的是中断文件里的值
    "  addi t2, zero, 0x71",          # 保留的选址
    "  csrrw zero, 0x350, t2",
    "  csrrs t3, 0x351, zero",
    "  sw   t3, 0(a0)",               # 0：保留选址读 0
    "  addi t2, zero, 0xC0",          # miselect = eie0
    "  csrrw zero, 0x350, t2",
    "  addi t2, zero, 0x20",          # 使能 5 号
    "  csrrw zero, 0x351, t2",
    "  csrrs t3, 0x35C, zero",
    "  sw   t3, 0(a0)",               # 0x00050005：mtopei 报 5 号
    "  csrrw t3, 0x35C, zero",        # 写即领取，读到的仍是领取前的值
    "  sw   t3, 0(a0)",               # 0x00050005
    "  csrrs t3, 0x35C, zero",
    "  sw   t3, 0(a0)",               # 0：领取之后没有待决的了
    "  jal  zero, imdone",
"imtrap:",
    "  csrrs t2, 0x341, zero",
    "  addi t2, t2, 4",
    "  csrrw zero, 0x341, t2",
    "  addi t3, zero, -2",
    "  sw   t3, 0(a0)",               # 哨兵：这条 CSR 访问陷入了
    "  mret",
"imdone:",
]
IMSIC_EXP = [0x70, 1, 0, 0x00050005, 0x00050005, 0]

NOCSR = [
    *vec("nocsr1"),
    "  csrrs t2, 0x7C0, zero",
"nocsr1:",
    "  csrrs t2, 0x342, zero",
    "  sw   t2, 0(a0)",
]
SMODE_OFF = [
    *vec("nos1"),
    "  addi t2, zero, 2",
    "  csrrs zero, 0x100, t2",
"nos1:",
    "  csrrs t2, 0x342, zero",
    "  sw   t2, 0(a0)",
]
SRC = HEAD + (MEXT if mul else []) + TAIL + (SMODE if smode else SMODE_OFF) + NOCSR
SRC += TRAPS
# imsic 这一段排在 DELEG 前面：DELEG 末尾在等软件中断，测试台按「对完最后一项」才送那一拍边沿，
# 接在它后面程序就停在那里等一个永远不来的中断（全开那一点 TIMEOUT 在第 69 项）
SRC += IMSIC if imsic else []
SRC += DELEG if smode else []
SRC += ["done:", "  jal  zero, done"]
EXPECT = (HEAD_EXP + (MEXT_EXP if mul else []) + TAIL_EXP
          + [2] + [2] + TRAPS_EXP + (IMSIC_EXP if imsic else [])
          + (DELEG_EXP if smode else []))

prog = assemble(SRC)
rom = "\n".join(f"      {i}: return 32'h{w:08X};" for i, w in enumerate(prog))
exp = "\n".join(f"      {i}: return 32'h{v:08X};" for i, v in enumerate(EXPECT))

# 假中断文件不按旋钮开关：核的窗口输入是 always_enabled，旋钮关掉时没人驱就报 G0066。
# 关掉时核根本不看这些输入，驱着也不花什么（照测试台驱 irq、pins 的写法）。
# imsic 开时这两块才真正被用到：只做这一刀要的三格（eidelivery 0x70、eip0 0x80、eie0 0xC0），
# 5 号恒挂起。领取把 5 号的待决位拿掉，再读就没有了
IM_REG = """
  Reg#(Bit#(32)) imDeliv <- mkReg(0);
  Reg#(Bit#(32)) imEie   <- mkReg(0);
  Reg#(Bit#(32)) imEip   <- mkReg(32'h0000_0020);
"""

# 读出与写入分两条规则：一条规则既驱 rdata 又读 wr 的话，核那一侧「读窗口」与「发写脉冲」
# 就绕成一个环，bsc 把文件这条排在前面，写脉冲永远看不见（G0010）
IM_RULE = """
  rule imsicRead;
    Bit#(8)  sel = cpu.imsic.sel;
    cpu.imsic.rdata((sel == 8'h70) ? imDeliv
                  : (sel == 8'h80) ? imEip
                  : (sel == 8'hC0) ? imEie : 0);
    // 最高号：投递开着、挂起且使能、过阈值（这一刀阈值恒放行）
    cpu.imsic.topei((imDeliv[0] == 1 && (imEip & imEie) != 0)
                    ? 32'h0005_0005 : 0);
  endrule

  rule imsicWrite;
    Bit#(8) sel = cpu.imsic.sel;
    // 写 eip 有两条来路（经窗口写、领取清位），并列的 if 各写一次就是并行冲突（G0004）：
    // 先算进局部变量，末尾只写一次
    Bit#(32) nEip = imEip;
    if (cpu.imsic.wr) begin
      if (sel == 8'h70) imDeliv <= cpu.imsic.wdata;
      else if (sel == 8'hC0) imEie <= cpu.imsic.wdata;
      else if (sel == 8'h80) nEip = cpu.imsic.wdata;
    end
    if (cpu.imsic.claim) nEip = nEip & ~32'h0000_0020;
    imEip <= nEip;
  endrule
"""

# rvfi 开时测试台多出的三块。记录比访存口晚一拍出来，所以最后一项对完不马上结束，等 8 拍再收账
RV_REG = """
  Reg#(Bit#(64)) rvCnt <- mkReg(0);
  Reg#(Bit#(32)) rvOut <- mkReg(0);
  Reg#(Bit#(32)) rvPc  <- mkReg(0);
  Reg#(Bool)     rvBad <- mkReg(False);
  Reg#(Bool)     done  <- mkReg(False);
  Reg#(Bit#(4))  lag   <- mkReg(0);
""" if rvfi else ""

RV_RULE = """
  // 三条判据：编号从零连续；前后两条的 pc 接得上（从陷入进来的那条带 intr，除外）；
  // 写自检口的记录地址、掩码、数据对得上期望值，条数等于自检项数
  rule rvCheck (cpu.rvfi.valid);
    let v = cpu.rvfi;
    Bool badOrder = v.order != rvCnt;
    Bool badPc    = rvCnt != 0 && !v.intr && v.pc_rdata != rvPc;
    Bool toOut    = v.mem_wmask != 0 && isOut(v.mem_addr);
    Bit#(32) want = expected(rvOut);
    Bool badOut   = toOut && (v.mem_addr != 32'h1000_0000 || v.mem_wmask != 4'hF || v.mem_wdata != want);
    rvCnt <= rvCnt + 1;
    rvPc  <= v.pc_wdata;
    if (toOut) rvOut <= rvOut + 1;
    // 三条并列的 if 各写一次 rvBad 在 bsc 看来是并行冲突（G0004），合成一次写
    if (badOrder || badPc || badOut) rvBad <= True;
    if (badOrder)
      $display("FAIL rvfi order: got %0d want %0d at pc %08h", v.order, rvCnt, v.pc_rdata);
    if (badPc)
      $display("FAIL rvfi record %0d: pc_rdata %08h but the previous pc_wdata was %08h",
               v.order, v.pc_rdata, rvPc);
    if (badOut)
      $display("FAIL rvfi record %0d: store a=%08h mask=%b data=%08h want %08h",
               v.order, v.mem_addr, v.mem_wmask, v.mem_wdata, want);
  endrule

  rule rvFin (done);
    lag <= lag + 1;
    if (lag == 8) begin
      Bool ok = !bad && !rvBad && rvOut == fromInteger(expLen);
      if (rvOut != fromInteger(expLen))
        $display("FAIL rvfi: %0d records wrote the check port, want %0d", rvOut, expLen);
      if (!ok) $display("FAILED");
      else $display("PASS all %0d checks in %0d cycles, %0d instructions, %0d rvfi records",
                    expLen, cyc, progLen, rvCnt);
      $finish(ok ? 0 : 1);
    end
  endrule
""" if rvfi else ""

RV_FIN = "          done <= True;\n" if rvfi else """          if (bad) $display("FAILED");
          else $display("PASS all %0d checks in %0d cycles, %0d instructions",
                        expLen, cyc, progLen);
          $finish(bad ? 1 : 0);
"""

(out / f"Prog{label}.bsv").write_text(f"""package Prog{label};

// 由 htest/mktb.py 生成，勿手改。改程序改那个脚本。
// 这一点：mul={mul} smode={smode} mmu={mmu}

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

(out / f"Hart{label}Tb.bsv").write_text(f'''package Hart{label}Tb;

import RegFile::*;
import ConfigReg::*;
import RegIf::*;
import Hart::*;
import RvfiPins::*;
import Prog{label}::*;

// 核的自检台。判据不是「跑起来了」而是「结果对不对」：程序把每个算式的结果
// 写到 0x1000_0000，这里按顺序对期望值。
// 这一点：mul={mul} smode={smode} mmu={mmu} rvfi={rvfi}

// 出问题时把它改成 True，每次访存都打出来。上一次逮到的就是这么逮到的：
// addi rd, rs, -1 被译成 sub，因为立即数型借用了 funct7。
Bool trace = False;

(* synthesize *)
module mkHart{label}Tb(Empty);
  HartIfc#(12, 32) cpu <- mkHart(HartCfg {{ mul: {"True" if mul else "False"},
                                            smode: {"True" if smode else "False"},
                                            mmu: {"True" if mmu else "False"},
                                            rvfi: {"True" if rvfi else "False"},
                                            imsic: {"True" if imsic else "False"} }});
  RegFile#(Bit#(8), Bit#(32)) ram <- mkRegFileFull;

  Reg#(Bit#(32)) cyc  <- mkReg(0);
  // tick 要读它，超时时好说停在第几项。普通寄存器会让 tick 与 dmem 互为先后成环
  // （dmem 读 tick 写的 cyc），dmem 于是被挡住，喂核的线报 G0066
  Reg#(Bit#(32)) seen <- mkConfigReg(0);
  Reg#(Bool)     bad  <- mkReg(False);
  Reg#(Bool)     sent <- mkReg(False);
{IM_REG}{RV_REG}
  function Bool inRom(Bit#(32) a)  = a[31:28] == 4'h8 && a[17:16] == 0;
  function Bool inRam(Bit#(32) a)  = a[31:28] == 4'h8 && a[17:16] == 1;
  function Bool inRoot(Bit#(32) a) = a[31:28] == 4'h8 && a[17:16] == 2;
  // RAM 下标取页号两位加页内六位：物理页 0x80010 与 0x80012 必须分得开，
  // 不然「换了翻译」读回来的还是同一格，那条判据就是空的
  function Bit#(8) ramIx(Bit#(32) a) = {{a[13:12], a[7:2]}};

  // 两个四兆大页恒等映射 0x1000_0000 与 0x8000_0000，0x4000_0000 指向 RAM 里的二级页表
  function Bit#(32) rootPte(Bit#(10) i);
    case (i)
      10'h040: return 32'h040000C7;
      10'h100: return 32'h20004C01;
      10'h101: return 32'h24000001;
      10'h200: return 32'h200000CF;
      default: return 0;
    endcase
  endfunction
  function Bool isOut(Bit#(32) a) = a[31:28] == 4'h1;

  rule tick;
    cyc <= cyc + 1;
    if (cyc > 20000) begin
      $display("TIMEOUT after %0d cycles, %0d checks passed", cyc, seen);
      $finish(1);
    end
  endrule

  // 取指口：ROM 组合读出
  rule fetch;
    Bit#(32) a = cpu.imem.req.addr;
    // 取指这一侧的 MMU 也走表，页表项从这个口读
    Bit#(32) w = inRom(a)  ? romWord((a - 32'h8000_0000) >> 2)
               : inRam(a)  ? ram.sub(ramIx(a))
               : inRoot(a) ? rootPte(a[11:2]) : 32'h00000013;
    cpu.imem.ready(cpu.imem.valid);
    cpu.imem.resp(cpu.imem.valid, RegRsp {{ rdata: w, err: False }});
  endrule

  // 访存口：RAM 与自检口
  rule dmem;
    let r = cpu.dmem.req;
    Bit#(32) rd = 0;
    if (cpu.dmem.valid) begin
      if (inRam(r.addr)) begin
        Bit#(8) i = ramIx(r.addr);
        Bit#(32) old = ram.sub(i);
        rd = old;
        if (r.write) ram.upd(i, applyStrb(old, r.wdata, r.wstrb));
        if (trace)
          $display("MEM %s a=%08h i=%0d strb=%b wd=%08h old=%08h",
                   r.write ? "W" : "R", r.addr, i, r.wstrb, r.wdata, old);
      end else if (inRoot(r.addr)) begin
        rd = rootPte(r.addr[11:2]);
      end else if (isOut(r.addr) && r.write) begin
        Bit#(32) want = expected(seen);
        if (r.wdata != want) begin
          $display("FAIL check %0d: got %08h want %08h", seen, r.wdata, want);
          bad <= True;
        end
        seen <= seen + 1;
        if (seen + 1 == fromInteger(expLen)) begin
{RV_FIN}        end
      end
    end
    cpu.dmem.ready(cpu.dmem.valid);
    // 0x9xxx_xxxx 是这颗测试芯片上不存在的地址，答错
    cpu.dmem.resp(cpu.dmem.valid, RegRsp {{ rdata: rd, err: r.addr[31:28] == 4'h9 }});
  endrule

  rule plat;
    // 只剩最后一项没对的时候送一拍 SSWI 的边沿——程序此刻正卡在 swait 上等它。
    // 按检查计数而不是按拍数，程序改长改短都不用重调。
    Bool edge_ = {"True" if smode else "False"} && !sent && seen == fromInteger(expLen - 1);
    if (edge_) sent <= True;
    cpu.irq.irq(False, False, False, edge_);
    cpu.pins.hartid(0);
    cpu.pins.halt(False);
  endrule
{IM_RULE}{RV_RULE}endmodule

endpackage
''', encoding="utf-8")
print(f"  程序 {len(prog)} 条指令，自检 {len(EXPECT)} 项（mul={mul} smode={smode} mmu={mmu} rvfi={rvfi} imsic={imsic}）")
