
`timescale 1ns/1ns

module testbench();
	
	
	reg clk, rst;
	reg signed [26:0]Cr, Ci;
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
		//Cr = 27'h800000;   //1
		//Ci = 27'h800000;   //1 
		//Cr = 27'b1111_01000000000000000000000;   //-0.75  
		//Ci = 27'b0001_01000000000000000000000;   //1.25
		//Cr = 27'h200000;  //0.25
		//Ci = 27'h200000;  //0.25
		//Cr = 27'b1110_00000000000000000000000;      //-2
		//Ci = 27'b1111_00000000000000000000000;      //-1   
		Cr = 27'b1110_10000000000000000000000;      //-3
		Ci = 27'b1111_11111000000000000000000;      //-3 
		//Cr = 27'b0000_00000000000000000000000;      //-3
		//Ci = 27'b0000_00000000000000000000000;      //-3 
 		rst  = 1'b0;
		#10 
		rst  = 1'b1;
		#30
		rst  = 1'b0;
	end
	
	mandelbrot_iterator DUT( .*
	);
	
	
endmodule
