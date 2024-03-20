module drum_33_33 (
    clk,
    rst,
    //rho,
    output_un,
    output_time
);
    input clk;
    input rst;
    //input [17:0] rho;
    output signed [17:0] output_un;
    output [31:0] output_time; 

    wire signed [17:0] side_node_u_n [32:0];
    wire signed [17:0] u_np1 [32:0];
    wire signed [17:0] drum_center_node;
    wire [31:0] timer;
    wire signed [17:0] rho_eff;

    //nonlinear rho calculation
    rho_calculaion rho_eff_cal (.rho_0(18'b0_0_1000_0000_0000_0000), // 0.25
                            .gten(18'b0_0_0010_0000_0000_0000), // 2^-4
                            .u_n_ij(drum_center_node),
                            .rho_eff(rho_eff)
                            );

    genvar i;
    generate 
        for(i = 0; i < 33; i=i+1) begin: genrate_columns
            if(i == 0) begin
                //the left most column
                column_simulation #(.NUM(0)) col_1 (.clk(clk),
                                                    .rst(rst),
                                                    .rho(rho_eff),
                                                    .u_np1_ij_out(u_np1[0]),
                                                    .left(18'b0),
                                                    .right(side_node_u_n[1]),
                                                    .u_n_ij_out(side_node_u_n[0]),
                                                    .drum_center_out(),
                                                    .output_time_out(timer)
                                                    );

            end else if(i == 32) begin
                //the right most column
                column_simulation #(.NUM(32)) col_2 (.clk(clk),
                                                    .rst(rst),
                                                    .rho(rho_eff),
                                                    .u_np1_ij_out(u_np1[32]),
                                                    .left(side_node_u_n[31]),
                                                    .right(18'b0),
                                                    .u_n_ij_out(side_node_u_n[32]),
                                                    .drum_center_out(),
                                                    .output_time_out()
                                                    );

            end else begin
                //the middle columns
                if(i == 16)begin
                    //find the center node
                    column_simulation #(.NUM(i)) col_3 (.clk(clk),
                                                    .rst(rst),
                                                    .rho(rho_eff),
                                                    .u_np1_ij_out(u_np1[i]),
                                                    .left(side_node_u_n[i-1]),
                                                    .right(side_node_u_n[i+1]),
                                                    .u_n_ij_out(side_node_u_n[i]),
                                                    .drum_center_out(drum_center_node),
                                                    .output_time_out()
                                                    );
                end else begin
                    column_simulation #(.NUM(i)) col_4 (.clk(clk),
                                                        .rst(rst),
                                                        .rho(rho_eff),
                                                        .u_np1_ij_out(u_np1[i]),
                                                        .left(side_node_u_n[i-1]),
                                                        .right(side_node_u_n[i+1]),
                                                        .u_n_ij_out(side_node_u_n[i]),
                                                        .drum_center_out(),
                                                        .output_time_out()
                                                        );
                end

            end
        end
    endgenerate

    //assign the output the middle node of the drum
    assign output_un = drum_center_node;

    assign output_time = timer;


endmodule

module rho_calculaion (
    rho_0,
    gten,
    u_n_ij,
    rho_eff
);

    input signed [17:0] rho_0;
    input signed [17:0] gten;
    input signed [17:0] u_n_ij;
    output signed [17:0] rho_eff;

    wire signed [17:0] a;
    wire signed [17:0] b;
  

    signed_mult aaa (.out(a),
                     .a(gten),
                     .b(u_n_ij)
                    );

    signed_mult bbb (.out(b),
                     .a(a),
                     .b(a)
                    );

    assign rho_eff = b + rho_0;

endmodule

module column_simulation #(parameter NUM = 0) (
    clk,
    rst,
    rho,
    u_np1_ij_out,
    left,
    right,
    u_n_ij_out,
    drum_center_out,
    output_time_out
);
    input clk;
    input rst;
    input [17:0] rho; 
    output signed [17:0] u_np1_ij_out;
    input signed [17:0] left;
    input signed [17:0] right;
    output signed [17:0] u_n_ij_out;
    output signed [17:0] drum_center_out;
    output [31:0] output_time_out;
    parameter [5:0] NUM_ROW = 6'd33;
   
    wire signed [17:0] u_np1_ij;
    wire signed [17:0] u_nm1_ij;
    wire signed [17:0] u_n_ijp1;

    reg [5:0] row; //we only have one column right so we traverse the rows
    reg signed [17:0] wt_data_u_n;
    reg signed [17:0] wt_data_u_nm1;
    reg signed [17:0] in_wt_data_u_n;
    reg signed [17:0] in_wt_data_u_nm1;
    reg signed [17:0] u_n_bot;
    reg signed [17:0] u_n_ij;
    reg signed [17:0] u_n_ijm1;
    reg we_un;
    reg we_unm1;
    reg signed [17:0] drum_center;
    reg [31:0] output_time;
    reg [31:0] output_time_fixed;
    assign row_out = row;
    //assign the output amplitude
    assign u_np1_ij_out = u_np1_ij;
    assign u_n_ij_out = u_n_ij;
    assign drum_center_out = drum_center;
    assign output_time_out = output_time_fixed;

    //implementation of the ComputeModule
    wire signed [17:0] input_u_n_ij;
    wire signed [17:0] input_u_n_i_jm1;
    wire signed [17:0] input_u_n_i_jp1;
    wire signed [17:0] input_left;
    wire signed [17:0] input_right;

    assign input_u_n_ij    [17:0] = (row == 18'b0)          ? u_n_bot : u_n_ij;
    assign input_u_n_i_jm1 [17:0] = (row == 18'b0)          ? 18'b0   : u_n_ijm1;
    assign input_u_n_i_jp1 [17:0] = (row + 6'b1 == NUM_ROW) ? 18'b0   : u_n_ijp1;
    assign input_left      [17:0] = (NUM == 0)              ? 18'b0   : left;     //the left most column
    assign input_right     [17:0] = (NUM == 32)             ? 18'b0   : right;    //the right most column

    ComputeModule_for_col compute_start (.u_n_ij(input_u_n_ij), 
                                         .u_nm1_ij(u_nm1_ij), 
                                         .u_n_im1_j(input_left), 
                                         .u_n_i_jm1(input_u_n_i_jm1), 
                                         .u_n_ip1_j(input_right), 
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
    reg time_done;
    wire input_we_un;
    wire input_we_unm1;
    
    //choose write enable signal between the initialization phase and process phase
    assign input_we_un   = (state == INITIAL_MEM || rst == 1)?ini_we_un   : we_un;
    assign input_we_unm1 = (state == INITIAL_MEM || rst == 1)?ini_we_unm1 : we_unm1;

    M10K_1000_8 un (.q(u_n_ijp1),
                    .d(in_wt_data_u_n),
                    .write_address(row),
                    .read_address(rd_addr), //read the next one
                    .we(input_we_un),
                    .clk(clk)
                    );

    M10K_1000_8 un_1 (.q(u_nm1_ij),
                      .d(in_wt_data_u_nm1),
                      .write_address(row),
                      .read_address(row), //read the current one
                      .we(input_we_unm1),
                      .clk(clk)
                      );

    always@(posedge clk)begin
        if(rst)begin
            state <= INITIAL_MEM;
            row <= 0;
            in_wt_data_u_n <= wt_data_u_n;
            in_wt_data_u_nm1 <= wt_data_u_nm1;
            ini_we_un <= 1;
            ini_we_unm1 <= 1;
            drum_center <= 0;
            output_time <= 0;
            output_time_fixed <= 0;
            time_done <= 0;
        end else begin
            case(state)
                INITIAL_MEM:begin
                    output_time <= output_time + 1;
                    row <= (row == NUM_ROW - 1)? row : row + 1;
                    in_wt_data_u_n <= wt_data_u_n;
                    in_wt_data_u_nm1 <= wt_data_u_nm1;
                    ini_we_un <= (row == NUM_ROW - 1)? 0: 1;
                    ini_we_unm1 <= (row == NUM_ROW - 1)? 0: 1;
                    output_time_fixed <= 0;
                    time_done <= 0;
                    state <= (row == NUM_ROW - 1) ? INITIAL:INITIAL_MEM;
                end
                INITIAL:begin
                    //reset to the row 0
                    output_time <= output_time + 1;
                    row           <= 0;
                    in_wt_data_u_n   <= 18'b0;
                    in_wt_data_u_nm1 <= 18'b0;
                    u_n_bot       <= 18'b0;
                    u_n_ij        <= 18'b0;
                    u_n_ijm1      <= 18'b0;
                    we_un         <= 0;
                    we_unm1       <= 0;
                    state         <= WAIT;
                end
                WAIT:begin
                    output_time <= output_time + 1;
                    //waiting for the data read from the memory block
                    state <= WRITE;
                end
                WRITE:begin
                    output_time <= output_time + 1;
                    //start writing to memory
                    //                               row 0  :  midlle rows and top row
                    in_wt_data_u_n   <= (row == 6'b0) ? in_wt_data_u_n : u_np1_ij ;
                    in_wt_data_u_nm1 <= (row == 6'b0) ? u_n_bot : u_n_ij;
                    u_n_ijm1      <= (row == 6'b0) ? u_n_bot : u_n_ij;
                    u_n_bot       <= (row == 6'b0) ? u_np1_ij: u_n_bot;
                    we_unm1       <= 1;
                    we_un         <= (row == 6'b0) ? 0 : 1;
                    state         <= SHIFT;
                    if(row == 6'd16 && NUM == 16) begin //midlle row
                        drum_center <= u_np1_ij;
                    end
                end
                SHIFT:begin
                    output_time <= output_time + 1;
                    //updating the un
                    u_n_ij  <= (row == NUM_ROW - 6'b1) ? u_n_bot : u_n_ijp1;
                    //reset the rows when reach the top row
                    row     <= (row == NUM_ROW - 6'b1) ? 6'b0 : row + 6'b1;
                    we_un   <= 0;
                    we_unm1 <= 0;
                    if(row == 6'd32) begin
                        time_done <= 1;
                    end
                    //start another traversal
                    state <= WAIT_2;
                end
                 WAIT_2:begin
                    output_time <= output_time + 1;
                    output_time_fixed <= (time_done && (output_time_fixed == 0)) ? output_time : output_time_fixed;
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
                //the first row of all columns is 0
                wt_data_u_n <= 18'b0;
                wt_data_u_nm1 <= 18'b0;
            end
            5'd1:begin //row number 2
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 4 || NUM == 28)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 5 || NUM == 27)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 7 || NUM == 25)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 9 || NUM == 23)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 11 || NUM == 21)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 13 || NUM == 19)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 15 || NUM == 17)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd2:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 4 || NUM == 28)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 6 || NUM == 26)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 8 || NUM == 24)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 10 || NUM == 22)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 12 || NUM == 20)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 14 || NUM == 18)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd3:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end 
                else if(NUM == 4 || NUM == 28)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 5 || NUM == 27)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 7 || NUM == 25)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 9 || NUM == 23)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 11 || NUM == 21)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 13 || NUM == 19)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 15 || NUM == 17)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd4:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end 
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd5:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                 else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end 
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd6:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                 else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end 
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd7:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                 else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end 
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd8:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                 else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end 
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd9:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                 else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end 
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd10:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end 
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd11:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd12:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd13:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0011_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0011_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0100_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0011_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0011_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0100_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd14:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0011_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0011_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_1000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0011_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_1000_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0011_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_1000_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd15:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0011_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0011_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_1000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0011_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_1100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0011_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_1100_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd16:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0011_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0011_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_1000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0011_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_1100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0100_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0100_0000_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd17:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0011_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0011_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_1000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0011_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_1100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0011_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_1100_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd18:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0011_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0011_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_1000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0011_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_1000_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0011_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_1000_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd19:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0011_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0011_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0100_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0011_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0011_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0100_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd20:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0011_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0011_0000_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd21:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0010_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1100_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd22:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0010_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_1000_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd23:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0010_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0100_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd24:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0010_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0010_0000_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd25:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0001_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1100_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd26:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0001_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_1000_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd27:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0001_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0100_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd28:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0001_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0001_0000_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd29:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0000_1100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1100_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd30:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0000_1000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_1000_0000_0000;
                end
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd31:begin
                if(NUM == 0 || NUM == 32)begin
                    wt_data_u_n <= 18'b0_0_0000_0000_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0000_0000_0000;
                end 
                else if(NUM == 1 || NUM == 31)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 2 || NUM == 30)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end 
                else if(NUM == 3 || NUM == 29)begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 4 || NUM == 28) begin
                   wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                   wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 5 || NUM == 27) begin
                   wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                   wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 6 || NUM == 26) begin
                   wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                   wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 7 || NUM == 25) begin
                   wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                   wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 8 || NUM == 24) begin
                   wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                   wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 9 || NUM == 23) begin
                   wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                   wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 10 || NUM == 22) begin
                   wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                   wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 11 || NUM == 21) begin
                   wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                   wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 12 || NUM == 20) begin
                   wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                   wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 13 || NUM == 19) begin
                   wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                   wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 14 || NUM == 18) begin
                   wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                   wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 15 || NUM == 17) begin
                   wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                   wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                else if(NUM == 16) begin
                    wt_data_u_n <= 18'b0_0_0000_0100_0000_0000;
                    wt_data_u_nm1 <= 18'b0_0_0000_0100_0000_0000;
                end
                
                else begin
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
                end
            end
            5'd32:begin
                  //the end row of all columns is 0
                    wt_data_u_n <= 18'b0;
                    wt_data_u_nm1 <= 18'b0;
              
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

