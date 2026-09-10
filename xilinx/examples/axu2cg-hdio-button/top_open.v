// Historical OpenXC7 form of the minimal button-to-LED test.
//
// This explicit placement workaround produced the first hardware-proven
// image.  run_open.sh now synthesizes the unconstrained behavioral top.v;
// nextpnr derives the LUT placement anchor from the HDIO output lane's route
// graph instead.  Keep this file only as a regression/reference artifact.
module top (
    input  wire key1_n,
    output wire led
);
    (* BEL = "SLICE_X14Y45/H6LUT" *)
    LUT1 #(
        .INIT(2'b01)
    ) key_to_led (
        .I0(key1_n),
        .O(led)
    );
endmodule
