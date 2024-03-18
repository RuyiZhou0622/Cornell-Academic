

module DE1_SoC_Computer (
	////////////////////////////////////
	// FPGA Pins
	////////////////////////////////////

	// Clock pins
	CLOCK_50,
	CLOCK2_50,
	CLOCK3_50,
	CLOCK4_50,

	// ADC
	ADC_CS_N,
	ADC_DIN,
	ADC_DOUT,
	ADC_SCLK,

	// Audio
	AUD_ADCDAT,
	AUD_ADCLRCK,
	AUD_BCLK,
	AUD_DACDAT,
	AUD_DACLRCK,
	AUD_XCK,

	// SDRAM
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_LDQM,
	DRAM_RAS_N,
	DRAM_UDQM,
	DRAM_WE_N,

	// I2C Bus for Configuration of the Audio and Video-In Chips
	FPGA_I2C_SCLK,
	FPGA_I2C_SDAT,

	// 40-Pin Headers
	GPIO_0,
	GPIO_1,
	
	// Seven Segment Displays
	HEX0,
	HEX1,
	HEX2,
	HEX3,
	HEX4,
	HEX5,

	// IR
	IRDA_RXD,
	IRDA_TXD,

	// Pushbuttons
	KEY,

	// LEDs
	LEDR,

	// PS2 Ports
	PS2_CLK,
	PS2_DAT,
	
	PS2_CLK2,
	PS2_DAT2,

	// Slider Switches
	SW,

	// Video-In
	TD_CLK27,
	TD_DATA,
	TD_HS,
	TD_RESET_N,
	TD_VS,

	// VGA
	VGA_B,
	VGA_BLANK_N,
	VGA_CLK,
	VGA_G,
	VGA_HS,
	VGA_R,
	VGA_SYNC_N,
	VGA_VS,

	////////////////////////////////////
	// HPS Pins
	////////////////////////////////////
	
	// DDR3 SDRAM
	HPS_DDR3_ADDR,
	HPS_DDR3_BA,
	HPS_DDR3_CAS_N,
	HPS_DDR3_CKE,
	HPS_DDR3_CK_N,
	HPS_DDR3_CK_P,
	HPS_DDR3_CS_N,
	HPS_DDR3_DM,
	HPS_DDR3_DQ,
	HPS_DDR3_DQS_N,
	HPS_DDR3_DQS_P,
	HPS_DDR3_ODT,
	HPS_DDR3_RAS_N,
	HPS_DDR3_RESET_N,
	HPS_DDR3_RZQ,
	HPS_DDR3_WE_N,

	// Ethernet
	HPS_ENET_GTX_CLK,
	HPS_ENET_INT_N,
	HPS_ENET_MDC,
	HPS_ENET_MDIO,
	HPS_ENET_RX_CLK,
	HPS_ENET_RX_DATA,
	HPS_ENET_RX_DV,
	HPS_ENET_TX_DATA,
	HPS_ENET_TX_EN,

	// Flash
	HPS_FLASH_DATA,
	HPS_FLASH_DCLK,
	HPS_FLASH_NCSO,

	// Accelerometer
	HPS_GSENSOR_INT,
		
	// General Purpose I/O
	HPS_GPIO,
		
	// I2C
	HPS_I2C_CONTROL,
	HPS_I2C1_SCLK,
	HPS_I2C1_SDAT,
	HPS_I2C2_SCLK,
	HPS_I2C2_SDAT,

	// Pushbutton
	HPS_KEY,

	// LED
	HPS_LED,
		
	// SD Card
	HPS_SD_CLK,
	HPS_SD_CMD,
	HPS_SD_DATA,

	// SPI
	HPS_SPIM_CLK,
	HPS_SPIM_MISO,
	HPS_SPIM_MOSI,
	HPS_SPIM_SS,

	// UART
	HPS_UART_RX,
	HPS_UART_TX,

	// USB
	HPS_CONV_USB_N,
	HPS_USB_CLKOUT,
	HPS_USB_DATA,
	HPS_USB_DIR,
	HPS_USB_NXT,
	HPS_USB_STP
);

//=======================================================
//  PARAMETER declarations
//=======================================================


//=======================================================
//  PORT declarations
//=======================================================

////////////////////////////////////
// FPGA Pins
////////////////////////////////////

// Clock pins
input						CLOCK_50;
input						CLOCK2_50;
input						CLOCK3_50;
input						CLOCK4_50;

// ADC
inout						ADC_CS_N;
output					ADC_DIN;
input						ADC_DOUT;
output					ADC_SCLK;

// Audio
input						AUD_ADCDAT;
inout						AUD_ADCLRCK;
inout						AUD_BCLK;
output					AUD_DACDAT;
inout						AUD_DACLRCK;
output					AUD_XCK;

// SDRAM
output 		[12: 0]	DRAM_ADDR;
output		[ 1: 0]	DRAM_BA;
output					DRAM_CAS_N;
output					DRAM_CKE;
output					DRAM_CLK;
output					DRAM_CS_N;
inout			[15: 0]	DRAM_DQ;
output					DRAM_LDQM;
output					DRAM_RAS_N;
output					DRAM_UDQM;
output					DRAM_WE_N;

// I2C Bus for Configuration of the Audio and Video-In Chips
output					FPGA_I2C_SCLK;
inout						FPGA_I2C_SDAT;

// 40-pin headers
inout			[35: 0]	GPIO_0;
inout			[35: 0]	GPIO_1;

// Seven Segment Displays
output		[ 6: 0]	HEX0;
output		[ 6: 0]	HEX1;
output		[ 6: 0]	HEX2;
output		[ 6: 0]	HEX3;
output		[ 6: 0]	HEX4;
output		[ 6: 0]	HEX5;

// IR
input						IRDA_RXD;
output					IRDA_TXD;

// Pushbuttons
input			[ 3: 0]	KEY;

// LEDs
output		[ 9: 0]	LEDR;

// PS2 Ports
inout						PS2_CLK;
inout						PS2_DAT;

inout						PS2_CLK2;
inout						PS2_DAT2;

// Slider Switches
input			[ 9: 0]	SW;

// Video-In
input						TD_CLK27;
input			[ 7: 0]	TD_DATA;
input						TD_HS;
output					TD_RESET_N;
input						TD_VS;

// VGA
output		[ 7: 0]	VGA_B;
output					VGA_BLANK_N;
output					VGA_CLK;
output		[ 7: 0]	VGA_G;
output					VGA_HS;
output		[ 7: 0]	VGA_R;
output					VGA_SYNC_N;
output					VGA_VS;



////////////////////////////////////
// HPS Pins
////////////////////////////////////
	
// DDR3 SDRAM
output		[14: 0]	HPS_DDR3_ADDR;
output		[ 2: 0]  HPS_DDR3_BA;
output					HPS_DDR3_CAS_N;
output					HPS_DDR3_CKE;
output					HPS_DDR3_CK_N;
output					HPS_DDR3_CK_P;
output					HPS_DDR3_CS_N;
output		[ 3: 0]	HPS_DDR3_DM;
inout			[31: 0]	HPS_DDR3_DQ;
inout			[ 3: 0]	HPS_DDR3_DQS_N;
inout			[ 3: 0]	HPS_DDR3_DQS_P;
output					HPS_DDR3_ODT;
output					HPS_DDR3_RAS_N;
output					HPS_DDR3_RESET_N;
input						HPS_DDR3_RZQ;
output					HPS_DDR3_WE_N;

// Ethernet
output					HPS_ENET_GTX_CLK;
inout						HPS_ENET_INT_N;
output					HPS_ENET_MDC;
inout						HPS_ENET_MDIO;
input						HPS_ENET_RX_CLK;
input			[ 3: 0]	HPS_ENET_RX_DATA;
input						HPS_ENET_RX_DV;
output		[ 3: 0]	HPS_ENET_TX_DATA;
output					HPS_ENET_TX_EN;

// Flash
inout			[ 3: 0]	HPS_FLASH_DATA;
output					HPS_FLASH_DCLK;
output					HPS_FLASH_NCSO;

// Accelerometer
inout						HPS_GSENSOR_INT;

// General Purpose I/O
inout			[ 1: 0]	HPS_GPIO;

// I2C
inout						HPS_I2C_CONTROL;
inout						HPS_I2C1_SCLK;
inout						HPS_I2C1_SDAT;
inout						HPS_I2C2_SCLK;
inout						HPS_I2C2_SDAT;

// Pushbutton
inout						HPS_KEY;

// LED
inout						HPS_LED;

// SD Card
output					HPS_SD_CLK;
inout						HPS_SD_CMD;
inout			[ 3: 0]	HPS_SD_DATA;

// SPI
output					HPS_SPIM_CLK;
input						HPS_SPIM_MISO;
output					HPS_SPIM_MOSI;
inout						HPS_SPIM_SS;

// UART
input						HPS_UART_RX;
output					HPS_UART_TX;

// USB
inout						HPS_CONV_USB_N;
input						HPS_USB_CLKOUT;
inout			[ 7: 0]	HPS_USB_DATA;
input						HPS_USB_DIR;
input						HPS_USB_NXT;
output					HPS_USB_STP;

//=======================================================
//  REG/WIRE declarations
//=======================================================

wire			[15: 0]	hex3_hex0;
//wire			[15: 0]	hex5_hex4;

//assign HEX0 = ~hex3_hex0[ 6: 0]; // hex3_hex0[ 6: 0]; 
//assign HEX1 = ~hex3_hex0[14: 8];
//assign HEX2 = ~hex3_hex0[22:16];
//assign HEX3 = ~hex3_hex0[30:24];
assign HEX4 = 7'b1111111;
assign HEX5 = 7'b1111111;
assign HEX3 = 7'b1111111;
assign HEX2 = 7'b1111111;
assign HEX1 = 7'b1111111;
assign HEX0 = 7'b1111111;

//=======================================================
// Audio controller for AVALON bus-master
//=======================================================
// computes DDS for sine wave and fills audio FIFO
// reads audio FIFO and loops it back
// MUST configure (in Qsys) Audio Config module:
// -- Line in to ADC
// -- uncheck both bypass options
// The audio_input_ready signal goes high for one
// cycle when there is new audio input data
// --
// 32-bit data is on 
//   right_audio_input, left_audio_input ;
// Every write requires 32-bit data on 
//   right_audio_output, left_audio_output ;

reg [31:0] bus_addr ; // Avalon address
// see 
// ftp://ftp.altera.com/up/pub/Altera_Material/15.1/University_Program_IP_Cores/Audio_Video/Audio.pdf
// for addresses
wire [31:0] audio_base_address = 32'h00003040 ;  // Avalon address
wire [31:0] audio_fifo_address = 32'h00003044 ;  // Avalon address +4 offset
wire [31:0] audio_left_address = 32'h00003048 ;  // Avalon address +8
wire [31:0] audio_right_address = 32'h0000304c ;  // Avalon address +12
reg [3:0] bus_byte_enable ; // four bit byte read/write mask
reg bus_read  ;       // high when requesting data
reg bus_write ;      //  high when writing data
reg [31:0] bus_write_data ; //  data to send to Avalog bus
wire bus_ack  ;       //  Avalon bus raises this when done
wire [31:0] bus_read_data ; // data from Avalon bus
reg [30:0] timer ;
reg [3:0] state ;
wire state_clock ;
wire reset;

// SW[9] disables state machine so that
// HPS has complete control of audio interface
//assign reset = ~KEY[0] | SW[9] ;

// current free words in audio interface
reg [7:0] fifo_space ;
// debug check of space
assign LEDR = fifo_space ;

// audio input/output from audio module FIFO
reg [15:0] right_audio_input, left_audio_input ;
reg audio_input_ready ;
wire [15:0] right_audio_output, left_audio_output ;

// For audio loopback, or filtering
//assign right_audio_output = SW[1]? right_filter_output : right_audio_input ;
//assign left_audio_output = SW[0]? decimated_filter_300_out : left_audio_input ;

// DDS update signal for testing
// reg [31:0] dds_accum ;
// // DDS LUT
// wire [15:0] sine_out ;
// update phase accumulator
// sync to audio data rate (48kHz) using audio_input_ready signal
// always @(posedge CLOCK_50) begin //CLOCK_50
// 	// Fout = (sample_rate)/(2^32)*{SW[9:0], 16'b0}
// 	// then Fout=48000/(2^32)*(2^25) = 375 Hz
// 	if (audio_input_ready) dds_accum <= dds_accum + {SW[9:0], 16'b0} ;	
// end
// DDS sine wave ROM
// sync_rom sineTable(CLOCK_50, dds_accum[31:24], sine_out);

// get some signals exposed
// connect bus master signals to i/o for probes
// assign GPIO_0[0] = bus_write ;
// assign GPIO_0[1] = bus_read ;
// assign GPIO_0[2] = decimated_audio_ready ;
// assign GPIO_0[3] = audio_input_ready ;

reg decimated_audio_ready ; 
reg [3:0] decimated_audio_clk_counter ;
reg request;
wire signed [17:0] out_un;
always @(posedge CLOCK_50) begin //CLOCK_50

	// reset state machine and read/write controls
	if (~KEY[0]) begin
		state <= 0 ;
		bus_read <= 0 ; // set to one if a read opeation from bus
		bus_write <= 0 ; // set to one if a write operation to bus
		timer <= 0;
		request <= 0;
	end
	else begin
		// timer just for deubgging
		timer <= timer + 1;
	end
	
	// set up read FIFO available space
	if (state==4'd0) begin
		bus_addr <= audio_fifo_address ;
		bus_read <= 1'b1 ;
		bus_byte_enable <= 4'b1111;
		state <= 4'd1 ; // wait for read ACK
	end
	
	// wait for read ACK and read the fifo available
	// bus ACK is high when data is available
	if (state==4'd1 && bus_ack==1) begin
		state <= 4'd2 ; //4'd2
		// FIFO space is in high byte
		fifo_space <= (bus_read_data>>24) ;
		// end the read
		bus_read <= 1'b0 ;
	end
	
	// When there is room in the FIFO
	// -- compute next DDS sine sample
	// -- start write to fifo for each channel
	// -- first the left channel
	if (state==4'd2 && fifo_space>8'd2) begin // 
		state <= 4'd3;	
		bus_write_data <= out_un << 14 ;
		request <= 1;
		bus_addr <= audio_left_address ;
		bus_byte_enable <= 4'b1111;
		bus_write <= 1'b1 ;
	end	
	// if no space, try again later
	else if (state==4'd2 && fifo_space<=8'd2) begin
		state <= 4'b0 ;
		request <= 0;
	end
	
	// detect bus-transaction-complete ACK 
	// for left channel write
	// You MUST do this check
	if (state==4'd3 && bus_ack==1) begin
		state <= 4'd4 ;
		bus_write <= 0;
		
	end
	
	// -- now the right channel
	if (state==4'd4) begin // 
		state <= 4'd5;	
		bus_write_data <= out_un << 14;
		bus_addr <= audio_right_address ;
		bus_write <= 1'b1 ;
	end	
	
	// detect bus-transaction-complete ACK
	// for right channel write
	// You MUST do this check
	if (state==4'd5 && bus_ack==1) begin
		state <= 4'd0 ;
		bus_write <= 0;
	end
	
end // always @(posedge state_clock)

//=======================================================
//  Structural coding
//=======================================================

Computer_System The_System (
	////////////////////////////////////
	// FPGA Side
	////////////////////////////////////

	// Global signals
	.system_pll_ref_clk_clk					(CLOCK_50),
	.system_pll_ref_reset_reset			(1'b0),
	.sdram_clk_clk								(state_clock),

	// AV Config
	.av_config_SCLK							(FPGA_I2C_SCLK),
	.av_config_SDAT							(FPGA_I2C_SDAT),

	// Audio Subsystem
	.audio_pll_ref_clk_clk					(CLOCK3_50),
	.audio_pll_ref_reset_reset				(1'b0),
	.audio_clk_clk								(AUD_XCK),
	.audio_ADCDAT								(AUD_ADCDAT),
	.audio_ADCLRCK								(AUD_ADCLRCK),
	.audio_BCLK									(AUD_BCLK),
	.audio_DACDAT								(AUD_DACDAT),
	.audio_DACLRCK								(AUD_DACLRCK),

	// bus-master state machine interface
	.bus_master_audio_external_interface_address     (bus_addr),     
	.bus_master_audio_external_interface_byte_enable (bus_byte_enable), 
	.bus_master_audio_external_interface_read        (bus_read),        
	.bus_master_audio_external_interface_write       (bus_write),       
	.bus_master_audio_external_interface_write_data  (bus_write_data),  
	.bus_master_audio_external_interface_acknowledge (bus_ack),                                  
	.bus_master_audio_external_interface_read_data   (bus_read_data),   
	
	
	////////////////////////////////////
	// HPS Side
	////////////////////////////////////
	// DDR3 SDRAM
	.memory_mem_a			(HPS_DDR3_ADDR),
	.memory_mem_ba			(HPS_DDR3_BA),
	.memory_mem_ck			(HPS_DDR3_CK_P),
	.memory_mem_ck_n		(HPS_DDR3_CK_N),
	.memory_mem_cke		(HPS_DDR3_CKE),
	.memory_mem_cs_n		(HPS_DDR3_CS_N),
	.memory_mem_ras_n		(HPS_DDR3_RAS_N),
	.memory_mem_cas_n		(HPS_DDR3_CAS_N),
	.memory_mem_we_n		(HPS_DDR3_WE_N),
	.memory_mem_reset_n	(HPS_DDR3_RESET_N),
	.memory_mem_dq			(HPS_DDR3_DQ),
	.memory_mem_dqs		(HPS_DDR3_DQS_P),
	.memory_mem_dqs_n		(HPS_DDR3_DQS_N),
	.memory_mem_odt		(HPS_DDR3_ODT),
	.memory_mem_dm			(HPS_DDR3_DM),
	.memory_oct_rzqin		(HPS_DDR3_RZQ),
		  
	// Ethernet
	.hps_io_hps_io_gpio_inst_GPIO35	(HPS_ENET_INT_N),
	.hps_io_hps_io_emac1_inst_TX_CLK	(HPS_ENET_GTX_CLK),
	.hps_io_hps_io_emac1_inst_TXD0	(HPS_ENET_TX_DATA[0]),
	.hps_io_hps_io_emac1_inst_TXD1	(HPS_ENET_TX_DATA[1]),
	.hps_io_hps_io_emac1_inst_TXD2	(HPS_ENET_TX_DATA[2]),
	.hps_io_hps_io_emac1_inst_TXD3	(HPS_ENET_TX_DATA[3]),
	.hps_io_hps_io_emac1_inst_RXD0	(HPS_ENET_RX_DATA[0]),
	.hps_io_hps_io_emac1_inst_MDIO	(HPS_ENET_MDIO),
	.hps_io_hps_io_emac1_inst_MDC		(HPS_ENET_MDC),
	.hps_io_hps_io_emac1_inst_RX_CTL	(HPS_ENET_RX_DV),
	.hps_io_hps_io_emac1_inst_TX_CTL	(HPS_ENET_TX_EN),
	.hps_io_hps_io_emac1_inst_RX_CLK	(HPS_ENET_RX_CLK),
	.hps_io_hps_io_emac1_inst_RXD1	(HPS_ENET_RX_DATA[1]),
	.hps_io_hps_io_emac1_inst_RXD2	(HPS_ENET_RX_DATA[2]),
	.hps_io_hps_io_emac1_inst_RXD3	(HPS_ENET_RX_DATA[3]),

	// Flash
	.hps_io_hps_io_qspi_inst_IO0	(HPS_FLASH_DATA[0]),
	.hps_io_hps_io_qspi_inst_IO1	(HPS_FLASH_DATA[1]),
	.hps_io_hps_io_qspi_inst_IO2	(HPS_FLASH_DATA[2]),
	.hps_io_hps_io_qspi_inst_IO3	(HPS_FLASH_DATA[3]),
	.hps_io_hps_io_qspi_inst_SS0	(HPS_FLASH_NCSO),
	.hps_io_hps_io_qspi_inst_CLK	(HPS_FLASH_DCLK),

	// Accelerometer
	.hps_io_hps_io_gpio_inst_GPIO61	(HPS_GSENSOR_INT),

	//.adc_sclk                        (ADC_SCLK),
	//.adc_cs_n                        (ADC_CS_N),
	//.adc_dout                        (ADC_DOUT),
	//.adc_din                         (ADC_DIN),

	// General Purpose I/O
	.hps_io_hps_io_gpio_inst_GPIO40	(HPS_GPIO[0]),
	.hps_io_hps_io_gpio_inst_GPIO41	(HPS_GPIO[1]),

	// I2C
	.hps_io_hps_io_gpio_inst_GPIO48	(HPS_I2C_CONTROL),
	.hps_io_hps_io_i2c0_inst_SDA		(HPS_I2C1_SDAT),
	.hps_io_hps_io_i2c0_inst_SCL		(HPS_I2C1_SCLK),
	.hps_io_hps_io_i2c1_inst_SDA		(HPS_I2C2_SDAT),
	.hps_io_hps_io_i2c1_inst_SCL		(HPS_I2C2_SCLK),

	// Pushbutton
	.hps_io_hps_io_gpio_inst_GPIO54	(HPS_KEY),

	// LED
	.hps_io_hps_io_gpio_inst_GPIO53	(HPS_LED),

	// SD Card
	.hps_io_hps_io_sdio_inst_CMD	(HPS_SD_CMD),
	.hps_io_hps_io_sdio_inst_D0	(HPS_SD_DATA[0]),
	.hps_io_hps_io_sdio_inst_D1	(HPS_SD_DATA[1]),
	.hps_io_hps_io_sdio_inst_CLK	(HPS_SD_CLK),
	.hps_io_hps_io_sdio_inst_D2	(HPS_SD_DATA[2]),
	.hps_io_hps_io_sdio_inst_D3	(HPS_SD_DATA[3]),

	// SPI
	.hps_io_hps_io_spim1_inst_CLK		(HPS_SPIM_CLK),
	.hps_io_hps_io_spim1_inst_MOSI	(HPS_SPIM_MOSI),
	.hps_io_hps_io_spim1_inst_MISO	(HPS_SPIM_MISO),
	.hps_io_hps_io_spim1_inst_SS0		(HPS_SPIM_SS),

	// UART
	.hps_io_hps_io_uart0_inst_RX	(HPS_UART_RX),
	.hps_io_hps_io_uart0_inst_TX	(HPS_UART_TX),

	// USB
	.hps_io_hps_io_gpio_inst_GPIO09	(HPS_CONV_USB_N),
	.hps_io_hps_io_usb1_inst_D0		(HPS_USB_DATA[0]),
	.hps_io_hps_io_usb1_inst_D1		(HPS_USB_DATA[1]),
	.hps_io_hps_io_usb1_inst_D2		(HPS_USB_DATA[2]),
	.hps_io_hps_io_usb1_inst_D3		(HPS_USB_DATA[3]),
	.hps_io_hps_io_usb1_inst_D4		(HPS_USB_DATA[4]),
	.hps_io_hps_io_usb1_inst_D5		(HPS_USB_DATA[5]),
	.hps_io_hps_io_usb1_inst_D6		(HPS_USB_DATA[6]),
	.hps_io_hps_io_usb1_inst_D7		(HPS_USB_DATA[7]),
	.hps_io_hps_io_usb1_inst_CLK		(HPS_USB_CLKOUT),
	.hps_io_hps_io_usb1_inst_STP		(HPS_USB_STP),
	.hps_io_hps_io_usb1_inst_DIR		(HPS_USB_DIR),
	.hps_io_hps_io_usb1_inst_NXT		(HPS_USB_NXT)
);


column_simulation one_col (.clk(CLOCK_50),
									.rst(~KEY[0]),
									.request(request),
									.rho(18'b0_0_0010_0000_0000_0000),
									.u_np1_ij_out(out_un)
									);


endmodule


module column_simulation(
    clk,
    rst,

    //  row_out,
    //  wt_data_u_n_in,
    //  wt_data_u_nm1_in,
    request,
    rho,
    u_np1_ij_out
);
    input clk;
    input rst;
    input [17:0] rho;
    input request;
    // output [4:0] row_out;
    // input signed [17:0] wt_data_u_n_in;
    // input signed [17:0] wt_data_u_nm1_in;
    output signed [17:0] u_np1_ij_out;
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

    // assign row_out = row;
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
        end else begin
            case(state)
                INITIAL_MEM:begin
                    row <= (row == NUM_ROW - 1)? row : row + 1;
                    in_wt_data_u_n <= wt_data_u_n;
                    in_wt_data_u_nm1 <= wt_data_u_nm1;
                    ini_we_un <= (row == NUM_ROW - 1)? 0: 1;
                    ini_we_unm1 <= (row == NUM_ROW - 1)? 0: 1;
                    state <= (row == NUM_ROW - 1) ? INITIAL:INITIAL_MEM;
                end
                INITIAL:begin
                    //reset to the row 0
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
                    //waiting for the data read from the memory block
                    state <= WRITE;
                end
                WRITE:begin
                    //start writing to memory
                    //                               row 0  :  midlle rows and top row
                    in_wt_data_u_n   <= (row == 6'b0) ? in_wt_data_u_n : u_np1_ij ;
                    in_wt_data_u_nm1 <= (row == 6'b0) ? u_n_bot : u_n_ij;
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
                    if(request)begin
                        //waiting for the data read from the memory block
                        state <= WAIT;
                    end else begin
                        state <= WAIT_2; //wait until the audio core is ready to take another input.
                    end
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

