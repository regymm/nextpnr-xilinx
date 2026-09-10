// Minimal ALINX AXU2CGA/B combinational HDIO input-to-output test.
module top (
    input  wire key1_n,
    output wire led
);
    // KEY1 is active low and the LED is active high.
    assign led = ~key1_n;
endmodule
