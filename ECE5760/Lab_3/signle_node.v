module ComputeModule(
    clk,
    rst,
    u_n_ij, 
    u_nm1_ij, 
    u_n_im1_j, 
    u_n_i_jm1, 
    u_n_i_jp1, 
    u_n_ip1_j, 
    rho,
    u_np1_ij  
);

    input clk;
    input rst;
    input signed [17:0] u_n_ij, u_nm1_ij;
    input signed [17:0] u_n_im1_j, u_n_i_jm1, u_n_i_jp1, u_n_ip1_j;  // Assuming 1.17 fixed 
    input [17:0] rho;
    output signed [17:0] u_np1_ij;  

    parameter ng_dt_2 = 18'b0_0_0000_0000_1000_0000; //2^(-10)

    wire [17:0] alpha;
    assign alpha = 18'b1_0_0000_0000_0000_0000 - ng_dt_2;   // 1 - (ng*dt)/2

    wire signed  [17:0] tmp1, tmp3;
    wire signed [17:0] rho_tmp1;
    wire signed [17:0] al_nm1ij;
    wire signed [17:0] al_tmp3;
    reg signed  [17:0] un, un_1;

    always@(posedge clk) begin
        if(rst) begin
            un_1 <= u_nm1_ij;
            un   <= u_n_ij;
        end else begin
            un_1  <= un;
            un    <= al_tmp3;

        end
    end

   
        
    assign tmp1 = (u_n_ip1_j - un) + (u_n_im1_j - un) + (u_n_i_jp1 - un) + (u_n_i_jm1 - un);
    assign tmp3 = (un <<1) + rho_tmp1 - al_nm1ij;
    assign u_np1_ij = al_tmp3;

      

    //implementing the fix-point multiplier
    signed_mult mul_1 (.out(rho_tmp1),
                       .a(rho),
                       .b(tmp1)
                       );

    signed_mult mul_2 (.out(al_nm1ij),
                       .a(alpha),
                       .b(un_1)
                       );
     
    signed_mult mul_3 (.out(al_tmp3),
                       .a(alpha),
                       .b(tmp3)
                       );





//////////////////////////////////////////////////
//// signed mult of 1.17 format 2'comp////////////
//////////////////////////////////////////////////
module signed_mult (out, a, b);
	output 	signed  [17:0]	out;
	input 	signed	[17:0] 	a;
	input 	signed	[17:0] 	b;
	// intermediate full bit length
	wire 	signed	[35:0]	mult_out;
	assign mult_out = a * b;
	// select bits for 7.20 fixed point
	assign out = {mult_out[35], mult_out[33:17]};
endmodule

endmodule
