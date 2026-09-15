// hart 的 riscv-formal 外壳：mkRvOn 是 mkHart 以 mul、smode、mmu 关，rvfi 开编出来的顶层。
// 取指口与访存口的响应时刻与数据都交给求解器，只约束契约本身：没有请求不给响应。
// err 恒为零、中断线恒为低：访存错与中断进入不在这一版里验。

module rvfi_wrapper (
	input         clock,
	input         reset,
	`RVFI_OUTPUTS
);
	(* keep *) `rvformal_rand_reg        imem_ready;
	(* keep *) `rvformal_rand_reg        imem_resp;
	(* keep *) `rvformal_rand_reg [31:0] imem_rdata;
	(* keep *) `rvformal_rand_reg        dmem_ready;
	(* keep *) `rvformal_rand_reg        dmem_resp;
	(* keep *) `rvformal_rand_reg [31:0] dmem_rdata;

	(* keep *) wire        imem_valid;
	(* keep *) wire [68:0] imem_req;
	(* keep *) wire        dmem_valid;
	(* keep *) wire [68:0] dmem_req;

	mkRvOn uut (
		.CLK          (clock),
		.RST_N        (!reset),

		.imem_valid   (imem_valid),
		.imem_req     (imem_req),
		.imem_ready_r (imem_ready),
		.imem_resp_v  (imem_valid && imem_resp),
		.imem_resp_x  ({imem_rdata, 1'b0}),

		.dmem_valid   (dmem_valid),
		.dmem_req     (dmem_req),
		.dmem_ready_r (dmem_ready),
		.dmem_resp_v  (dmem_valid && dmem_resp),
		.dmem_resp_x  ({dmem_rdata, 1'b0}),

		.irq_msip     (1'b0),
		.irq_mtip     (1'b0),
		.irq_meip     (1'b0),
		.irq_ssip_set (1'b0),
		.pins_hartid  (32'd0),
		.pins_halt    (1'b0),

		`RVFI_CONN32
	);

`ifdef RISCV_FORMAL_FAIRNESS
	// liveness 与 hang 两项由 genchecks 定义这个宏：要求目标迟早答，一笔请求最多等两拍
	reg [1:0] istall = 0;
	reg [1:0] dstall = 0;
	always @(posedge clock) begin
		istall <= imem_valid && !imem_resp ? istall + 1 : 0;
		dstall <= dmem_valid && !dmem_resp ? dstall + 1 : 0;
	end
	always @* begin
		if (istall == 2) assume (imem_resp);
		if (dstall == 2) assume (dmem_resp);
	end
`endif
endmodule
