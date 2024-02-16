
`timescale 1ns/1ns

module testbench();
	
	
	reg clk, rst;
	reg [26:0]Cr, Ci;
	wire[9:0] iteration ;
	
	//Initialize clocks and index
	initial begin
		clk = 1'b0;
	end
	
	//Toggle the clocks
	always begin
		#10
		clk  = !clk;
	end
	
	//Intialize and drive signals
	initial begin
		//Cr = 27'h800000;
		//Ci = 27'h800000;
		//Cr = 27'hA00000;  //1.25
		//Ci = 27'hA00000;  //1.25
		Cr = 27'h200000;  //1.25
		Ci = 27'h200000;  //1.25
		//Cr = 0;
		//Ci = 0;
		rst  = 1'b0;
		#10 
		rst  = 1'b1;
		#30
		rst  = 1'b0;
	end
	
	mandelbrot_iterator DUT( .*
	);
	
	
endmodule
