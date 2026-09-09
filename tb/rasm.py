"""一个够用的 RV32I[M] 汇编器，给核的测试台生成程序。

不引外部工具链：测试台要的只是几十条指令，而多一个外部依赖就多一处
「在我机器上能跑」。有了它，测试用例可以直接写成 Python 列表。
"""
from __future__ import annotations

R = {f"x{i}": i for i in range(32)}
R.update({"zero": 0, "ra": 1, "sp": 2, "gp": 3, "tp": 4,
          "t0": 5, "t1": 6, "t2": 7, "s0": 8, "s1": 9,
          **{f"a{i}": 10 + i for i in range(8)},
          **{f"s{i}": 16 + i for i in range(2, 12)},
          **{f"t{i}": 25 + i for i in range(3, 7)}})

RTYPE = {  # funct7, funct3
    "add": (0x00, 0), "sub": (0x20, 0), "sll": (0x00, 1), "slt": (0x00, 2),
    "sltu": (0x00, 3), "xor": (0x00, 4), "srl": (0x00, 5), "sra": (0x20, 5),
    "or": (0x00, 6), "and": (0x00, 7),
    "mul": (0x01, 0), "mulh": (0x01, 1), "mulhsu": (0x01, 2), "mulhu": (0x01, 3),
    "div": (0x01, 4), "divu": (0x01, 5), "rem": (0x01, 6), "remu": (0x01, 7),
}
ITYPE = {"addi": 0, "slti": 2, "sltiu": 3, "xori": 4, "ori": 6, "andi": 7}
SHIFT = {"slli": (0x00, 1), "srli": (0x00, 5), "srai": (0x20, 5)}
LOAD = {"lb": 0, "lh": 1, "lw": 2, "lbu": 4, "lhu": 5}
STORE = {"sb": 0, "sh": 1, "sw": 2}
BRANCH = {"beq": 0, "bne": 1, "blt": 4, "bge": 5, "bltu": 6, "bgeu": 7}
CSRI = {"csrrw": 1, "csrrs": 2, "csrrc": 3,
        "csrrwi": 5, "csrrsi": 6, "csrrci": 7}


def _u(v: int, n: int) -> int:
    return v & ((1 << n) - 1)


def asm_one(line: str, pc: int, labels: dict[str, int]) -> int:
    parts = line.replace(",", " ").split()
    op, a = parts[0], parts[1:]

    def reg(s):
        if s not in R:
            raise ValueError(f"未知寄存器 {s}")
        return R[s]

    def imm(s):
        # hi()/lo() 取标签的绝对地址：陷入向量要的是地址本身，不是相对量
        if s.startswith(('hi(', 'lo(')) and s.endswith(')'):
            v = labels[s[3:-1]]
            if s[0] == 'h':
                return (v + 0x800) >> 12
            v &= 0xFFF
            return v - 0x1000 if v & 0x800 else v
        return labels[s] - pc if s in labels else int(s, 0)

    if op in RTYPE:
        f7, f3 = RTYPE[op]
        return (f7 << 25) | (reg(a[2]) << 20) | (reg(a[1]) << 15) \
            | (f3 << 12) | (reg(a[0]) << 7) | 0x33
    if op in ITYPE:
        return (_u(imm(a[2]), 12) << 20) | (reg(a[1]) << 15) \
            | (ITYPE[op] << 12) | (reg(a[0]) << 7) | 0x13
    if op in SHIFT:
        f7, f3 = SHIFT[op]
        return (f7 << 25) | (_u(imm(a[2]), 5) << 20) | (reg(a[1]) << 15) \
            | (f3 << 12) | (reg(a[0]) << 7) | 0x13
    if op in LOAD:
        off, base = a[1].rstrip(")").split("(")
        return (_u(int(off, 0), 12) << 20) | (reg(base) << 15) \
            | (LOAD[op] << 12) | (reg(a[0]) << 7) | 0x03
    if op in STORE:
        off, base = a[1].rstrip(")").split("(")
        v = _u(int(off, 0), 12)
        return ((v >> 5) << 25) | (reg(a[0]) << 20) | (reg(base) << 15) \
            | (STORE[op] << 12) | ((v & 0x1F) << 7) | 0x23
    if op in BRANCH:
        v = _u(imm(a[2]), 13)
        return (((v >> 12) & 1) << 31) | (((v >> 5) & 0x3F) << 25) \
            | (reg(a[1]) << 20) | (reg(a[0]) << 15) | (BRANCH[op] << 12) \
            | (((v >> 1) & 0xF) << 8) | (((v >> 11) & 1) << 7) | 0x63
    if op == "lui":
        return (_u(imm(a[1]), 20) << 12) | (reg(a[0]) << 7) | 0x37
    if op == "auipc":
        return (_u(imm(a[1]), 20) << 12) | (reg(a[0]) << 7) | 0x17
    if op == "jal":
        v = _u(imm(a[1]), 21)
        return (((v >> 20) & 1) << 31) | (((v >> 1) & 0x3FF) << 21) \
            | (((v >> 11) & 1) << 20) | (((v >> 12) & 0xFF) << 12) \
            | (reg(a[0]) << 7) | 0x6F
    if op == "jalr":
        off, base = a[1].rstrip(")").split("(")
        return (_u(int(off, 0), 12) << 20) | (reg(base) << 15) \
            | (reg(a[0]) << 7) | 0x67
    if op in CSRI:
        src = reg(a[2]) if CSRI[op] < 4 else _u(int(a[2], 0), 5)
        return (_u(int(a[1], 0), 12) << 20) | (src << 15) \
            | (CSRI[op] << 12) | (reg(a[0]) << 7) | 0x73
    if op == "ecall":
        return 0x73
    if op == "ebreak":
        return 0x00100073
    if op == "mret":
        return 0x30200073
    if op == "sret":
        return 0x10200073
    if op == "nop":
        return 0x13
    raise ValueError(f"不认识的指令 {op}")


def assemble(src: list[str], base: int = 0x8000_0000) -> list[int]:
    """两趟：先收标签，再编码。标签写成 `name:` 单独一行。"""
    labels, body = {}, []
    pc = base
    for line in src:
        s = line.split("#")[0].strip()
        if not s:
            continue
        if s.endswith(":"):
            labels[s[:-1]] = pc
            continue
        body.append((pc, s))
        pc += 4
    return [asm_one(s, p, labels) for p, s in body]
