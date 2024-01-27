//in the top-level module ///////////////////
module top_module(clk, reset, funct, Initial_x, Initial_y, Initial_z, out_x, out_y, out_z, rho, sigma, beta, CLOCK_50);

input clk, reset, CLOCK_50;
input signed [26:0] funct, rho, sigma, beta;  
input signed [26:0] Initial_x, Initial_y, Initial_z;  
output signed [26:0] out_x, out_y, out_z; 

wire signed [26:0] out_x_wire, out_y_wire, out_z_wire, pre_new_z; 
wire signed [26:0] pre_funct_x;
wire signed [26:0] funct_x, funct_y, funct_z;

assign out_x = out_x_wire;
assign out_y = out_y_wire;
assign out_z = out_z_wire;

// clock divider to slow system down for testing
reg [4:0] count;
	// analog update divided clock
	always @ (posedge CLOCK_50) 
		begin
				count <= count + 1; 
		end	
	assign AnalogClock = (count==0);		

	//--------------------------- implementation of X------------------------------------//
	
	assign pre_funct_x = out_y_wire - out_x_wire;
	

	// rho * v1
	signed_mult for_x (.out(funct_x),
					   .a(pre_funct_x),
					   .b(sigma)
					  );

	//instantiation of integrator for x
	integrator output_x (.out(out_x_wire),
						 .funct(funct_x),
						 .InitialOut(Initial_x),
						 .clk(clk),
						 .reset(reset)
						);

	//--------------------------- implementation of Y------------------------------------//
	
	assign pre_new_z = rho - out_z_wire;
	

	// Xk * (Zk - rho)
	signed_mult for_y (.out(new_z),
					   	 .a(pre_new_z),
					     .b(out_x_wire)
					  );

	assign funct_y = new_z - out_y;

	integrator output_y(.out(out_y_wire),
						 .funct(funct_y),
						 .InitialOut(Initial_y),
						 .clk(clk),
						 .reset(reset)
						);

	//--------------------------- implementation of Z------------------------------------//
	// Zk * beta
	signed_mult for_z_1 (.out(new_out_z),
					   	 .a(out_z_wire),
					     .b(beta)
					  	);

	// Xk * Yk
	signed_mult for_z_2 (.out(x_y),
					   	 .a(out_x_wire),
					     .b(out_z_wire)
					  	);

	assign funct_z = x_y - new_out_z;

	integrator output_z(.out(out_z_wire),
						 .funct(funct_z),
						 .InitialOut(Initial_z),
						 .clk(clk),
						 .reset(reset)
						);


		
endmodule

/////////////////////////////////////////////////
//// integrator /////////////////////////////////
/////////////////////////////////////////////////

module integrator(out,funct,InitialOut,clk,reset);
	output signed [26:0] out; 		//the state variable V
	input signed [26:0] funct;      //the dV/dt function
	input clk, reset;
	input signed [26:0] InitialOut;  //the initial state variable V
	
	wire signed	[26:0] out, v1new, funct_new ;
	reg signed	[26:0] v1 ;
	
	always @ (posedge clk) 
	begin
		if (reset==0) //reset	
			v1 <= InitialOut ; // 
		else 
			v1 <= v1new ;	
	end
	assign v1new = v1 + funct_new ;
	assign out = v1 ;

	signed_mult multiplier_1 (.out(funct_new),
							  .a(funct),
							  .b(27'b0000000_00000001_00000000000)
							  );

endmodule

//////////////////////////////////////////////////
//// signed mult of 7.20 format 2'comp////////////
//////////////////////////////////////////////////

module signed_mult (out, a, b);
	output 	signed  [26:0]	out;
	input 	signed	[26:0] 	a;
	input 	signed	[26:0] 	b;
	// intermediate full bit length
	wire 	signed	[53:0]	mult_out;
	assign mult_out = a * b;
	// select bits for 7.20 fixed point
	assign out = {mult_out[53], mult_out[45:20]};
endmodule
//////////////////////////////////////////////////

