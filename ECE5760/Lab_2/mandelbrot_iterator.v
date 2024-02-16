module mandelbrot_iterator (
    clk,
    rst,
    Cr,
    Ci,
    iteration
);
    
    input [26:0] Cr;
    input [26:0] Ci;
    input clk;
    input rst;
    output unsigned [9:0] iteration;

    parameter IDLE = 2'b00;
    parameter CALC = 2'b01;
    parameter DONE = 2'b10;
    parameter MAX = 10'd1000;

    reg signed [26:0] Z_N_r_sq;
    reg signed [26:0] Z_N_i_sq;
    reg signed [26:0] Z_N_r;
    reg signed [26:0] Z_N_i;

    wire signed [26:0] Z_N_r_sq_temp;
    wire signed [26:0] Z_N_i_sq_temp;
    wire signed [26:0] Z_N_r_next;
    wire signed [26:0] Z_N_i_next;
    wire signed [26:0] Z_NR_temp;
    wire signed [26:0] Z_NRI_temp;


    wire signed [26:0] Z_NR;
    
    reg  [9:0] num_iterations;
    reg  [9:0] out_num;
    wire signed [26:0] result;
    reg  [1:0] state;
    reg  [1:0] next_state;
    
    //output the number of iterations
    assign iteration = out_num;

    always @(posedge clk or posedge rst)begin
        state <= next_state;
        if(rst)begin
            next_state <= IDLE;
            num_iterations <= 1;
            Z_N_r_sq <= 0;
            Z_N_i_sq <= 0;
            Z_N_r <= 0;
            Z_N_i <= 0;
            out_num <= 0;
        end
        else begin
            case(state)
                IDLE: begin
                    next_state <= CALC;
                end
                CALC: begin
                    if(result >  26'h2000000)begin
                        num_iterations <= num_iterations;
                    end else begin
                        num_iterations <= num_iterations + 1;
                    end
                    Z_N_r_sq <= Z_N_r_sq_temp;
                    Z_N_i_sq <= Z_N_i_sq_temp;
                    Z_N_r <= Z_NR_temp;
                    Z_N_i <= Z_NRI_temp;
                    if (result > 26'h2000000 || num_iterations >= 10'd999) begin
                
                        next_state <= DONE;
                    end else begin
                        //next_state <= CALC;
                    end
                end
                DONE: begin
                    out_num <= num_iterations-1;
                    next_state <= DONE;
                end
            default: begin next_state <= IDLE;
                           num_iterations <= 0;
                     end
            endcase
        end
    end

    // always@(posedge clk or posedge rst)begin
    //     if(rst)begin
    //         num_iterations <= 0;
    //         Z_N_r_sq <= 0;
    //         Z_N_i_sq <= 0;
    //         Z_N_r <= 0;
    //         Z_N_i <= 0;

    //     end else begin
    //         num_iterations <= num_iterations + 1;

    //         Z_N_r_sq <= Z_N_r_sq_temp;
    //         Z_N_i_sq <= Z_N_i_sq_temp;
    //         Z_N_r <= Z_NR_temp;
    //         Z_N_i <= Z_NRI_temp;
    //     end


    // end


    assign Z_NR_temp = Z_N_r_sq - Z_N_i_sq + Cr;
   // assign Z_N_r_sq_temp = Z_NR_temp * Z_NR_temp;
    unsigned_mult mul_1 (.out(Z_N_r_sq_temp),
                       .a(Z_NR_temp),
                       .b(Z_NR_temp)
                       );
                
    //assign Z_NRI_temp = ((Z_N_r * Z_N_i) << 1) + Ci;
    wire signed  [26:0]	mul_out;
    wire signed  [26:0] mul_out_shifted;
    assign mul_out_shifted = mul_out << 1;
    signed_mult mul_2 (.out(mul_out),
                       .a(Z_N_r),
                       .b(Z_N_i)
                       );
  //  assign Z_NRI_temp = (mul_out << 1) + Ci;
    assign Z_NRI_temp = ({mul_out[26],mul_out_shifted[25:0]}) + Ci;

    //assign Z_N_i_sq_temp = Z_NRI_temp * Z_NRI_temp;
    unsigned_mult mul_3 (.out(Z_N_i_sq_temp),
                       .a(Z_NRI_temp),
                       .b(Z_NRI_temp)
                       );

    assign result = Z_N_i_sq_temp + Z_N_r_sq_temp ;
    
endmodule

//////////////////////////////////////////////////
//// signed mult of 4.23 format 2'comp////////////
//////////////////////////////////////////////////

module signed_mult (out, a, b);
	output 	signed  [26:0]	out;
	input 	signed	[26:0] 	a;
	input 	signed	[26:0] 	b;
	// intermediate full bit length
	wire 	signed	[53:0]	mult_out;
	assign mult_out = a * b;
	// select bits for 7.20 fixed point
	assign out = {mult_out[53], mult_out[48:23]};
endmodule

//////////////////////////////////////////////////
//// unsigned mult of 4.23 format 2'comp////////////
//////////////////////////////////////////////////

module unsigned_mult (out, a, b);
	output 	unsigned  [26:0]	out;
	input 	signed	[26:0] 	a;
	input 	signed	[26:0] 	b;
	// intermediate full bit length
	wire 	signed	[53:0]	mult_out;
	assign mult_out = a * b;
	// select bits for 7.20 fixed point
	assign out = mult_out[49:23];
endmodule



