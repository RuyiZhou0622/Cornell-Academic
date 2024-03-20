`timescale 1ns/1ps

module drum_33_33_tb;

reg clk;
reg rst;
//reg [17:0] rho;
wire signed [17:0]  output_un;
wire [31:0]  output_time;



// Instantiate the Drum Module
drum_33_33 DUT (
    .*
);

// Clock generation
initial begin
    clk = 0;
    forever #5 clk = ~clk; // Generate a clock with 10 ns period
end

// Test sequence
initial begin
    // Initialize inputs
    rst = 1;
   // rho       = 18'b0_0_0010_0000_0000_0000;
    // Apply reset
    #17;
    rst = 0; // Release reset

  
end

endmodule
