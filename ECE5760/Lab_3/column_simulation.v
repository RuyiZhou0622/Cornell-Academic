module column_simulation(
    clk,
    rst,
    rho,
    u_np1_ij_out
);
    input clk;
    input rst;
    input [17:0] rho;
    output signed [17:0] u_np1_ij_out;
    parameter [5:0] NUM_ROW = 6'd33;
   
    wire signed [17:0] current_u_n_ijp1;
    wire signed [17:0] initial_u_n_ijp1;
    wire signed [17:0] current_u_nm1_ij;
    wire signed [17:0] initial_u_nm1_ij;
    wire signed [17:0] u_np1_ij;
    wire signed [17:0] u_nm1_ij;
    wire signed [17:0] u_n_ijp1;

    assign u_n_ijp1 = (initial_read) ? initial_u_n_ijp1 : current_u_n_ijp1; //choose the initial traversal or others
    assign u_nm1_ij = (initial_read) ? initial_u_nm1_ij : current_u_nm1_ij; //choose the initial traversal or others

    //assign the output amplitude
    assign u_np1_ij_out = u_np1_ij;

    //instantiation of the memory blocks
    wire[5:0] rd_addr;
    assign rd_addr = (row+1 == NUM_ROW) ? 5'b0 : row + 1;
    M10K_1000_8 un (.q(current_u_n_ijp1),
                    .d(wt_data_u_n),
                    .write_address(row),
                    .read_address(rd_addr), //read the next one
                    .we(we_un),
                    .clk(clk)
                    );

    M10K_1000_8 un_1 (.q(current_u_nm1_ij),
                      .d(wt_data_u_nm1),
                      .write_address(row),
                      .read_address(row), //read the current one
                      .we(we_unm1),
                      .clk(clk)
                      );

    M10K_1000_8_ini init_un (.q(initial_u_n_ijp1),
                         .d(1'b0),
                         .write_address(1'b0),
                         .read_address(rd_addr), //read the next one
                         .we(1'b0),  //do not write anything
                         .clk(clk)
                         );

    M10K_1000_8_ini init_un_1 (.q(initial_u_nm1_ij),
                           .d(1'b0),
                           .write_address(1'b0),
                           .read_address(row), //read the current one
                           .we(1'b0), //do not write anything
                           .clk(clk)
                           );

    //implementation of the ComputeModule
    wire signed [17:0] input_u_n_ij;
    wire signed [17:0] input_u_n_i_jm1;
    wire signed [17:0] input_u_n_i_jp1;

    assign input_u_n_ij    [17:0] = (row == 18'b0) ? u_n_bot : u_n_ij;
    assign input_u_n_i_jm1 [17:0] = (row == 18'b0) ? 18'b0 : u_n_ijm1;
    assign input_u_n_i_jp1 [17:0] = (row + 5'b1 == NUM_ROW) ? 18'b0 : u_n_ijp1;

    ComputeModule_for_col compute_start (.u_n_ij(input_u_n_ij), 
                                         .u_nm1_ij(u_nm1_ij), 
                                         .u_n_im1_j(18'b0), 
                                         .u_n_i_jm1(input_u_n_i_jm1), 
                                         .u_n_ip1_j(18'b0), 
                                         .u_n_i_jp1(input_u_n_i_jp1), 
                                         .rho(rho),
                                         .u_np1_ij(u_np1_ij)  
                                         );

    //implementation of the FSM
    parameter [2:0] INITIAL  = 3'b000;
    parameter [2:0] WAIT   = 3'b001;
    parameter [2:0] WRITE    = 3'b010;
    parameter [2:0] SHIFT    = 3'b011;
    parameter [2:0] TRAVERSE = 3'b100;

    reg [5:0] row; //we only have one column right so we traverse the rows
    reg signed [17:0] wt_data_u_n;
    reg signed [17:0] wt_data_u_nm1;
    reg signed [17:0] u_n_bot;
    reg signed [17:0] u_n_ij;
    reg signed [17:0] u_n_ijm1;
    reg we_un;
    reg we_unm1;
    reg initial_read;

    reg [2:0] state;

    always@(posedge clk)begin
        if(rst)begin
            state <= 3'b000;
        end else begin
            case(state)
                INITIAL:begin
                    //reset to the row 0
                    row <= 0;
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                    u_n_bot <= 18'b0;
                    u_n_ij <= 18'b0;
                    u_n_ijm1 <= 18'b0;
                    we_un <= 0;
                    we_unm1 <= 0;
                    initial_read <= 1; 
                    state <= WRITE;
                end
                WRITE:begin
                    //start writing to memory
                    if(row == 5'b0)begin
                        //row 33
                        wt_data_u_nm1 <= u_n_bot;
                        u_n_ijm1 <= u_n_bot;
                        u_n_bot <= u_np1_ij;
                        we_unm1 <= 1;
                        we_un <= 0;

                    end else begin
                        //midlle rows and top row
                        wt_data_u_n <= u_np1_ij;
                        wt_data_u_nm1 <= u_n_ij;
                        u_n_ijm1 <= u_n_ij;
                        we_unm1 <= 1;
                        we_un <= 1;
                        u_n_bot <= u_n_bot;
                    end
                    state <= SHIFT;
                end
                SHIFT:begin
                    if( row + 5'b1 >= NUM_ROW)begin
                        //row 33
                      u_n_ij <= u_n_bot;
                      state <= TRAVERSE;
                      row <= 5'b0;
                    end else begin
                        //midlle rows
                        u_n_ij <= u_n_ijp1;
                        state <= WAIT;
                        row <= row + 5'b1;
                    end
                    we_un   <= 0;
                    we_unm1 <= 0;
                end
                TRAVERSE:begin
                    //finish the first travese for the column
                    initial_read <= 1'b0;
                    state <= WAIT;
                end
                
            endcase
        end
    end
endmodule

module ComputeModule_for_col(
    u_n_ij, 
    u_nm1_ij, 
    u_n_im1_j, 
    u_n_i_jm1, 
    u_n_ip1_j, 
    u_n_i_jp1, 
    rho,
    u_np1_ij  
);

    input signed [17:0] u_n_ij;
    input signed [17:0] u_nm1_ij;
    input signed [17:0] u_n_im1_j; 
    input signed [17:0] u_n_i_jm1; 
    input signed [17:0] u_n_i_jp1; 
    input signed [17:0] u_n_ip1_j;  // Assuming 1.17 fixed 
    input [17:0] rho;
    output signed [17:0] u_np1_ij;  

    parameter ng_dt_2 = 18'b0_0_0000_0000_1000_0000; //2^(-10)

    wire [17:0] alpha;
    assign alpha = 18'b1_0_0000_0000_0000_0000 - ng_dt_2;   // 1 - (ng*dt)/2

    wire signed  [17:0] tmp1, tmp3;
    wire signed [17:0] rho_tmp1;
    wire signed [17:0] al_nm1ij;
    wire signed [17:0] al_tmp3;
   // reg signed  [17:0] un, un_1;

    assign tmp1 = (u_n_ip1_j - u_n_ij) + (u_n_im1_j - u_n_ij) + (u_n_i_jp1 - u_n_ij) + (u_n_i_jm1 - u_n_ij);
    assign tmp3 = (u_n_ij <<1) + rho_tmp1 - al_nm1ij;
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
    output reg signed [17:0] q,
    input signed      [17:0] d,
    input [5:0] write_address, read_address,
    input we, clk
);

    reg signed [17:0] mem [32:0]; /* synthesis ramstyle = "no_rw_check, M10K" */

    always @ (posedge clk) begin
        if (we) begin
            mem[write_address] <= d;
		  end
        q <= mem[read_address]; // q doesn't get d in this clock cycle
    end
endmodule


module M10K_1000_8_ini( 
    output reg signed [17:0] q,
    input signed      [17:0] d,
    input [5:0] write_address, read_address,
    input we, clk
);

    reg signed [17:0] mem [32:0]; /* synthesis ramstyle = "no_rw_check, M10K" */

    initial begin
        $readmemb("initial_file.txt", mem);
    end
	 
    always @ (posedge clk) begin
        if (we) begin
            mem[write_address] <= d;
		  end
        q <= mem[read_address]; // q doesn't get d in this clock cycle
    end
endmodule
