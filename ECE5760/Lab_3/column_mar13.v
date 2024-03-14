module fsm_ComputeModule(
    clk,
    rst,
    //u_n_ij,   delete this input
    //u_nm1_ij, 
    u_n_im1_j, 
    //u_n_i_jm1,    delete this input
    //u_n_i_jp1, 
    u_n_ip1_j, 
    rho,
    u_np1_ij,
    u_n_bottom
);
    input clk;
    input rst;
    input signed [17:0]   u_n_bottom;
    input signed [17:0] u_n_im1_j, u_n_ip1_j;  // Assuming 1.17 fixed 
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
    reg init;
    reg signed [17:0] out_un_reg, out_un_1_reg, u_nm1_ij, u_n_i_jp1;

    wire signed  [17:0] tmp1, tmp3;
    wire signed [17:0] rho_tmp1;
    wire signed [17:0] al_nm1ij;
    wire signed [17:0] al_tmp3;
    reg signed  [17:0] unn, unn_1;

    parameter ng_dt_2 = 18'b0_0_0000_0000_1000_0000; //2^(-10)

    wire [17:0] alpha;
    assign alpha = 18'b1_0_0000_0000_0000_0000 - ng_dt_2;   // 1 - (ng*dt)/2

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
    reg [4:0] counter, counter_mem; 
    parameter [17:0] increment_mem = 18'b0_0_0000_0100_0001_1000;


    reg [17:0] u_n_i_jm1, u_n_i_j, u_n_i_j_reg, u_n_i_jm1_reg;
    parameter [2:0] MEM_INI = 3'b000;
    parameter [2:0] IDLE    = 3'b001;
    parameter [2:0] CALC    = 3'b010;
    parameter [2:0] DONE    = 3'b011;
    parameter [2:0] MEM    = 3'b011;

    always@(posedge clk)begin
        state <= next_state;
        if(rst)begin
            w_addr_un  <= 5'b11111;
            w_addr_un_1<= 5'b11111;
            r_addr_un  <= 5'b11111;
            r_addr_un_1<= 5'b11111;
            we_un      <= 1;
            we_un_1    <= 1;
            in_un      <= 0;
            in_un_1    <= 0;
            counter    <= 0;
            counter_mem<= 0;
            //un   <= u_n_ij;
            next_state <= MEM_INI;
        end else begin
            case(next_state)

                MEM_INI:begin
                    w_addr_un  <= w_addr_un + 1;
                    w_addr_un_1<= w_addr_un_1 + 1;
                    we_un      <= 1;
                    we_un_1    <= 1;
                    if(counter_mem <= 15) begin
                        in_un      <= increment_mem + in_un;
                        in_un_1    <= increment_mem + in_un_1;
                    end else begin
                        in_un      <= in_un - increment_mem;
                        in_un_1    <= in_un_1 - increment_mem;
                    end
                    
                    counter_mem    <= counter_mem + 1;
                    counter <= 0;
                    r_addr_un  <= 0;
                    r_addr_un_1<= 5'b11111;
                    init = 1;
                    next_state <= (counter_mem >= 5'd30)? IDLE : MEM_INI;
                    u_n_i_j <= u_n_bottom;
                    if(counter_mem == 30)begin
                        r_addr_un <= 5'b0;
                        r_addr_un_1 <= 5'b11111;
                    end  
                    //un_1 <= u_nm1_ij;
                end
                IDLE:begin

                    r_addr_un  <= r_addr_un + 1;
                    r_addr_un_1<= r_addr_un_1 + 1;

                    // counter <= (counter == 5'd30) ? 5'd0 : counter + 1;
                    we_un     <= 0;
                    we_un_1    <= 0;
                    //out_un_reg <= out_un;
                    u_nm1_ij   <= out_un_1; 
                    // un_1 <= u_nm1_ij;
                    // un   <= u_n_ij;

                    if(r_addr_un_1 == 0) begin
                        u_n_i_jm1 <= 0;
                        // u_n_i_j <= u_n_bottom;
                    end else begin
                        u_n_i_jm1 <= u_n_i_jm1_reg;
                        // u_n_i_j <= u_n_i_j_reg;
                    end

                    if (init == 1)begin
                        u_n_i_j <= u_n_bottom;
                    end else begin
                        u_n_i_j <= u_n_i_j_reg;
                    end
                    
                    if (r_addr_un_1 == 5'd30) begin
                        u_n_i_jp1 <= 0;
                    end else begin
                        u_n_i_jp1 <= out_un;
                    end
                    // r_addr_un  <= r_addr_un + 1;
                    // r_addr_un_1<= r_addr_un_1 + 1;
                    next_state <= CALC;
                end
                CALC:begin
                    next_state <= DONE;
                    //out_un
                end
                DONE: begin
                    
                    w_addr_un  <= r_addr_un_1;
                    w_addr_un_1<= r_addr_un_1;
         
                    we_un     <= 1;
                    we_un_1    <= 1;
                    in_un <= u_np1_ij;
                    in_un_1 <= u_n_i_jp1;
                    u_n_i_j_reg <= u_n_i_jp1;
                    u_n_i_jm1_reg <= u_n_i_j;
                    next_state <= IDLE;
                    init = 0;

                end
                default:next_state <= MEM_INI;
            endcase
        end
    end

    assign tmp1 = (u_n_ip1_j - u_n_i_j) + (u_n_im1_j - u_n_i_j) + (u_n_i_jp1 - u_n_i_j) + (u_n_i_jm1 - u_n_i_j);
    assign tmp3 = (u_n_i_j <<1) + rho_tmp1 - al_nm1ij;
    assign u_np1_ij = al_tmp3;

    //implementing the fix-point multiplier
    signed_mult mul_1 (.out(rho_tmp1),
                       .a(rho),
                       .b(tmp1)
                       );

    signed_mult mul_2 (.out(al_nm1ij),
                       .a(alpha),
                       .b(u_nm1_ij)
                       );
     
    signed_mult mul_3 (.out(al_tmp3),
                       .a(alpha),
                       .b(tmp3)
                       );

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
    reg signed  [17:0] unt, un_1t;

    always@(posedge clk) begin
        if(rst) begin
            un_1t <= u_nm1_ij;
            unt   <= u_n_ij;
        end else begin
            un_1t  <= unt;
            unt    <= al_tmp3;

        end
    end

   
        
    assign tmp1 = (u_n_ip1_j - unt) + (u_n_im1_j - unt) + (u_n_i_jp1 - unt) + (u_n_i_jm1 - unt);
    assign tmp3 = (unt <<1) + rho_tmp1 - al_nm1ij;
    assign u_np1_ij = al_tmp3;

      

    //implementing the fix-point multiplier
    signed_mult mul_1 (.out(rho_tmp1),
                       .a(rho),
                       .b(tmp1)
                       );

    signed_mult mul_2 (.out(al_nm1ij),
                       .a(alpha),
                       .b(un_1t)
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

