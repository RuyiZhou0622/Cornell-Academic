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

    reg [5:0] row; //we only have one column right so we traverse the rows
    reg signed [17:0] wt_data_u_n;
    reg signed [17:0] wt_data_u_nm1;
    reg signed [17:0] u_n_bot;
    reg signed [17:0] u_n_ij;
    reg signed [17:0] u_n_ijm1;
    reg we_un;
    reg we_unm1;

    //assign the output amplitude
    assign u_np1_ij_out = u_np1_ij;

    //implementation of the ComputeModule
    wire signed [17:0] input_u_n_ij;
    wire signed [17:0] input_u_n_i_jm1;
    wire signed [17:0] input_u_n_i_jp1;

    assign input_u_n_ij    [17:0] = (row == 18'b0)          ? u_n_bot : u_n_ij;
    assign input_u_n_i_jm1 [17:0] = (row == 18'b0)          ? 18'b0   : u_n_ijm1;
    assign input_u_n_i_jp1 [17:0] = (row + 6'b1 == NUM_ROW) ? 18'b0   : u_n_ijp1;

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
    parameter [2:0] INITIAL_MEM = 3'b000;
    parameter [2:0] INITIAL     = 3'b001;
    parameter [2:0] WAIT        = 3'b010;
    parameter [2:0] WRITE       = 3'b011;
    parameter [2:0] SHIFT       = 3'b100;
    parameter [2:0] WAIT_2      = 3'b101;

    //instantiation of the memory blocks
    wire[5:0] rd_addr;
    reg ini_we_un;
    reg ini_we_unm1;
    assign rd_addr = (row+1 == NUM_ROW) ? 5'b0 : row + 1;
    
    reg [2:0] state;
    wire input_we_un;
    wire input_we_unm1;
    
    //choose write enable signal between the initialization phase and process phase
    assign input_we_un   = (state == INITIAL_MEM || rst == 1)?ini_we_un   : we_un;
    assign input_we_unm1 = (state == INITIAL_MEM || rst == 1)?ini_we_unm1 : we_unm1;

    M10K_1000_8 un (.q(u_n_ijp1),
                    .d(wt_data_u_n),
                    .write_address(row),
                    .read_address(rd_addr), //read the next one
                    .we(input_we_un),
                    .clk(clk)
                    );

    M10K_1000_8 un_1 (.q(u_nm1_ij),
                      .d(wt_data_u_nm1),
                      .write_address(row),
                      .read_address(row), //read the current one
                      .we(input_we_unm1),
                      .clk(clk)
                      );

    always@(posedge clk)begin
        if(rst)begin
            state <= INITIAL_MEM;
            row <= 0;
            ini_we_un <= 1;
            ini_we_unm1 <= 1;
        end else begin
            case(state)
                INITIAL_MEM:begin
                    row <= (row == NUM_ROW - 1)? row : row + 1;
                    ini_we_un <= (row == NUM_ROW - 1)? 0: 1;
                    ini_we_unm1 <= (row == NUM_ROW - 1)? 0: 1;
                    state <= (row == NUM_ROW - 1) ? INITIAL:INITIAL_MEM;
                end
                INITIAL:begin
                    //reset to the row 0
                    row           <= 0;
                    wt_data_u_n   <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                    u_n_bot       <= 18'b0;
                    u_n_ij        <= 18'b0;
                    u_n_ijm1      <= 18'b0;
                    we_un         <= 0;
                    we_unm1       <= 0;
                    state         <= WAIT;
                end
                WAIT:begin
                    //waiting for the data read from the memory block
                    state <= WRITE;
                end
                WRITE:begin
                    //start writing to memory
                    //                               row 0  :  midlle rows and top row
                    wt_data_u_n   <= (row == 6'b0) ? wt_data_u_n : u_np1_ij ;
                    wt_data_u_nm1 <= (row == 6'b0) ? u_n_bot : u_n_ij;
                    u_n_ijm1      <= (row == 6'b0) ? u_n_bot : u_n_ij;
                    u_n_bot       <= (row == 6'b0) ? u_np1_ij: u_n_bot;
                    we_unm1       <= 1;
                    we_un         <= (row == 6'b0) ? 0 : 1;
                    state         <= SHIFT;
                end
                SHIFT:begin
                    //updating the un
                    u_n_ij  <= (row == NUM_ROW - 6'b1) ? u_n_bot : u_n_ijp1;
                    //reset the rows when reach the top row
                    row     <= (row == NUM_ROW - 6'b1) ? 6'b0 : row + 6'b1;
                    we_un   <= 0;
                    we_unm1 <= 0;
                    //start another traversal
                    state <= WAIT_2;
                end
                 WAIT_2:begin
                    //waiting for the data read from the memory block
                    state <= WRITE;
                end
            endcase
        end
    end

    //Look-Up Table for the initialization of the memory blocks
    always@(*)begin
        case(row)
            5'd0:begin
                        wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                    end
            5'd1:begin
                        wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                    end
            5'd2:begin
                        wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                    end
            5'd3:begin
                        wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                    end
            5'd4:begin
                        wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                    end
            5'd5:begin
                        wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                    end
            5'd6:begin
                        wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                    end
            5'd7:begin
                        wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                    end
            5'd8:begin
                        wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                    end
            5'd9:begin
                        wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                    end
            5'd10:begin
                        wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                    end
            5'd11:begin
                        wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                    end
            5'd12:begin
                        wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                    end
            5'd13:begin
                        wt_data_u_n <= 18'b0_0_0011_0100_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0011_0100_0000_0000;
                    end
            5'd14:begin
                        wt_data_u_n <= 18'b0_0_0011_1000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0011_1000_0000_0000;
                    end
            5'd15:begin
                        wt_data_u_n <= 18'b0_0_0011_1100_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0011_1100_0000_0000;
                    end
            5'd16:begin
                        wt_data_u_n <= 18'b0_0_0100_0000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0100_0000_0000_0000;
                    end
            5'd17:begin
                        wt_data_u_n <= 18'b0_0_0011_1100_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0011_1100_0000_0000;
                    end
            5'd18:begin
                        wt_data_u_n <= 18'b0_0_0011_1000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0011_1000_0000_0000;
                    end
            5'd19:begin
                        wt_data_u_n <= 18'b0_0_0011_0100_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0011_0100_0000_0000;
                    end
            5'd20:begin
                        wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                    end
            5'd21:begin
                        wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                    end
            5'd22:begin
                        wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                    end
            5'd23:begin
                        wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                    end
            5'd24:begin
                        wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                    end
            5'd25:begin
                        wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                    end
            5'd26:begin
                        wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                    end
            5'd27:begin
                        wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                    end
            5'd28:begin
                        wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                    end
            5'd29:begin
                        wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                    end
            5'd30:begin
                        wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                    end
            5'd31:begin
                        wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                    end
            5'd32:begin
                        wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                        wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                    end
            default:begin
                wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
            end
        endcase
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
