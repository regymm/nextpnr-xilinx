// Minimal AXU2CG clock, fabric counter, and HDIO output test.
module top (
    input  wire pl_ref_clk,
    output wire led
);
    wire pl_ref_clk_i;
    wire clk;

    IBUF refclk_ibuf (
        .I(pl_ref_clk),
        .O(pl_ref_clk_i)
    );

    BUFGCE refclk_bufg (
        .I(pl_ref_clk_i),
        .CE(1'b1),
        .O(clk)
    );

    reg [23:0] counter = 24'b0;
    always @(posedge clk)
        counter <= counter + 1'b1;

    assign led = counter[23];
endmodule
