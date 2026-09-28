# rvcore

Our RISC-V core in Bluespec: RV32IM, with supervisor mode and an Sv32 MMU as features.

![maturity](https://img.shields.io/badge/maturity-simulated-yellow) ![license](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0%20OR%20MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`xirang`](https://github.com/Tape-Out/xirang). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Simulated. Three stages that stall rather than bypass, a four-cycle multiplier and a bit-serial divider. The core module keeps the RISC-V name, `mkHart`.

| Feature | What it adds |
|:--:|:--|
| `mul` | the M extension (on by default) |
| `smode` | supervisor mode: delegation, supervisor trap state, `sret` |
| `mmu` | Sv32 translation with a 16-entry TLB and a two-level walker; needs `smode` |
| `rvfi` | the RVFI retirement port for riscv-formal |
| `imsic` | the AIA indirect CSRs and the interrupt file window |

There are no A or C extensions yet; Linux on a multicore SoC needs the first. A self-checking program runs at every point of the configuration matrix. Upstream's riscv-tests `rv32ui` and `rv32um` run in the physical environment (`htest/mkisatb.py`) at the points where `mmu`, `rvfi` and `imsic` are off, since that environment exercises none of them. `ma_data` is left out because misaligned accesses trap here, which the specification allows.

## License

任选其一：

- [MIT](LICENSE-MIT)
- [Apache 2.0](LICENSE-APACHE)
- [木兰宽松许可证 第2版](LICENSE-MULAN)

`SPDX-License-Identifier: MIT OR Apache-2.0 OR MulanPSL-2.0`

除非另行说明，你提交的贡献按上述三者同时授权，不附加其他条件。
