`timescale 1ns/1ps

module column_simulation_tb;

reg clk;
reg rst;
reg [17:0] rho;
reg signed [17:0] wt_data_u_n_in;
reg signed [17:0] wt_data_u_nm1_in;
wire [4:0] row_out;
wire signed [17:0]  u_np1_ij_out;

wire signed [17:0]  data_un;
wire signed [17:0]  data_unm1;

// Instantiate the ComputeModule
column_simulation DUT (
    .wt_data_u_n_in(data_un),
    .wt_data_u_nm1_in (data_unm1),
    .*
);

initial_value_LUT ini(.row(row_out),
                      .wt_data_u_n_out(data_un),
                      .wt_data_u_nm1_out(data_unm1)
                      );

// Clock generation
initial begin
    clk = 0;
    forever #5 clk = ~clk; // Generate a clock with 10 ns period
end

// Test sequence
initial begin
    // Initialize inputs
    rst = 1;
    rho       = 18'b0_0_0010_0000_0000_0000;
    // Apply reset
    #17;
    rst = 0; // Release reset

  
end

endmodule

module initial_value_LUT(
    row,
    wt_data_u_n_out,
    wt_data_u_nm1_out
);
    input [4:0] row;
    output signed [17:0] wt_data_u_n_out;
    output signed [17:0] wt_data_u_nm1_out;

    reg signed [17:0] wt_data_u_n;
    reg signed [17:0] wt_data_u_nm1;

    assign wt_data_u_n_out = wt_data_u_n;
    assign wt_data_u_nm1_out = wt_data_u_nm1;


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
