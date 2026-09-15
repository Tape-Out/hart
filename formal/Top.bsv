package Top;

// riscv-formal 验的顶层：RV32I、M 态，mul、smode、mmu 都关，rvfi 开。
// checks.cfg 的 verilog-files 列的是它编出来的 mkRvOn.v；编法：
//   bsc -verilog -u -g mkRvOn -p <生成的寄存器组>:../bsv:<hwcore>/bsv:<rvfi>/bsv:+ Top.bsv
// mkRvOn.v 不例化任何 bsc 原语，外壳加它两个文件就是全部

import Hart::*;

(* synthesize *)
module mkRvOn(HartIfc#(12, 32));
  let h <- mkHart(HartCfg { mul: False, smode: False, mmu: False, rvfi: True });
  return h;
endmodule

endpackage
