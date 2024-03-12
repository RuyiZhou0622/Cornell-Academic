module fsm_ComputeModule(
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

    // memory blocks initialization
    reg [4:0] w_addr_un;
    reg [4:0] w_addr_un_1;
    reg [4:0] r_addr_un;
    reg [4:0] r_addr_un_1;
    reg we_un;
    reg we_un_1;
    reg [17:0] in_un;
    wire [17:0] out_un;
    reg [17:0] in_un_1;
    wire [17:0] out_un_1;

    M10K_1000_8 un   (.q(out_un), //output
                      .d(in_un),
                      .write_address(w_addr_un),
                      .read_address(r_addr_un),
                      .we(we_un),
                      .clk(clk)
                      );

    M10K_1000_8 un_1 (.q(out_un_1), //output
                      .d(in_un_1),
                      .write_address( w_addr_un_1),
                      .read_address(r_addr_un_1),
                      .we(we_un_1),
                      .clk(clk)
                      );

    reg [2:0] state;
    reg [2:0] next_state;
    reg [4:0] counter; 

    parameter [2:0] MEM_INI = 3'b000;
    parameter [2:0] IDLE    = 3'b001;
    parameter [2:0] EMPTY   = 3'b010;

    always@(posedge clk)begin
        state <= next_state;
        if(rst)begin
            w_addr_un  <= 0;
            w_addr_un_1<= 0;
            r_addr_un  <= 0;
            r_addr_un_1<= 0;
            we_un      <= 1;
            we_un_1    <= 1;
            in_un      <= 0;
            in_un_1    <= 0;
            counter    <= 0;
            next_state <= MEM_INI;
        end else begin
            case(state)
                MEM_INI:begin
                    w_addr_un  <= w_addr_un + 1;
                    w_addr_un_1<= w_addr_un_1 + 1;
                    we_un      <= 1;
                    we_un_1    <= 1;
                    in_un      <= w_addr_un;
                    in_un_1    <= w_addr_un_1;
                    counter    <= counter + 1;
                    next_state <= (counter+3 >= 5'd30)? IDLE : MEM_INI;
                end
                IDLE:begin
                    counter <= (counter == 5'd30) ? 5'd0 : counter + 1;
                    we_un     <= 0;
                    we_un_1    <= 0;
                    r_addr_un  <= r_addr_un + 1;
                    r_addr_un_1<= r_addr_un_1 + 1;
                    next_state <= EMPTY;
                end
                EMPTY:begin
                    next_state <= IDLE;
                end
                default:next_state <= MEM_INI;
            endcase
        end
    end

endmodule


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






endmodule

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


//============================================================
// M10K module for testing
//============================================================
// See example 12-16 in 
// http://people.ece.cornell.edu/land/courses/ece5760/DE1_SOC/HDL_style_qts_qii51007.pdf
//============================================================
module M10K_1000_8( 
    output reg [17:0] q,
    input [17:0] d,
    input [4:0] write_address, read_address,//half
    input we, clk
);

    reg [17:0] mem [31:0]; /* synthesis ramstyle = "no_rw_check, M10K" */
	 
    always @ (posedge clk) begin
        if (we) begin
            mem[write_address] <= d;
		  end
        q <= mem[read_address]; // q doesn't get d in this clock cycle
    end
endmodule

