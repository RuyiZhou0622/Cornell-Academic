`timescale 1ns/1ps

module ComputeModule_tb;

reg clk;
reg rst;
reg signed  [17:0] u_n_ij, u_nm1_ij, u_n_im1_j, u_n_i_jm1, u_n_i_jp1, u_n_ip1_j;
reg         [17:0] rho;
wire signed [17:0] u_np1_ij;

// Instantiate the ComputeModule
ComputeModule uut (
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
    u_n_ij    = 18'b0_0_0100_0000_0000_0000;
    u_nm1_ij  = 18'b0_0_0100_0000_0000_0000;
    u_n_im1_j = 18'b0; 
    u_n_i_jm1 = 18'b0; 
    u_n_i_jp1 = 18'b0; 
    u_n_ip1_j = 18'b0;
    rho       = 18'b0_0_0010_0000_0000_0000;
    // Apply reset
    #17;
    rst = 0; // Release reset

  
end

endmodule
