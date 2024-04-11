///////////////////////////////////////
/// 640x480 version! 16-bit color
/// This code will segfault the original
/// DE1 computer
/// compile with
/// gcc lab3_control.c -o gr -O2 -lm
///
///////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h> 
#include <sys/shm.h> 
#include <sys/mman.h>
#include <sys/time.h> 
#include <math.h>
#include <pthread.h>
#include <math.h>

//define the parameters
#define ROWS 85
#define COLS 100
#define H    0.125


#define TRUE 1
#define FALSE 0
// video display
#define SDRAM_BASE            0xC0000000
#define SDRAM_END             0xC3FFFFFF
#define SDRAM_SPAN			  0x04000000
// characters
#define FPGA_CHAR_BASE        0xC9000000 
#define FPGA_CHAR_END         0xC9001FFF
#define FPGA_CHAR_SPAN        0x00002000
/* Cyclone V FPGA devices */
#define HW_REGS_BASE          0xff200000
//#define HW_REGS_SPAN        0x00200000 
#define HW_REGS_SPAN          0x00005000 

//-----------------PIO defines--------------------------------//
#define PIO_ROW               0x00000000
#define PIO_COL               0x00000010
#define PIO_DATA              0x00000020
#define PIO_RST               0x00000030  //in
#define PIO_RHOGAIN           0x00000040
#define PIO_NUMROW            0x00000050
//------------------------------------------------------------//

// the light weight buss base
void *h2p_lw_virtual_base;

// /dev/mem file id
int fd;

// define the data converter for fix point 1.17
typedef signed int fix;
#define int2fix(x)   (fix) (x <<17);
#define float2fix(x) (fix)(x * 131072.0);

#define fix2float(x) ((float)(x) / 131072.0);
#define fix2int(x)   (int) (x >> 17);

// === need to mmap: =======================
// FPGA_CHAR_BASE
// FPGA_ONCHIP_BASE      
// HW_REGS_BASE        
volatile unsigned int *pio_row = NULL;
volatile unsigned int *pio_col = NULL;
volatile signed int   *pio_data = NULL;
volatile unsigned int *pio_rst = NULL;
volatile signed int   *pio_rho_gain = NULL;
volatile unsigned int *pio_numrow = NULL;

//
int min(int a, int b){
    if (a > b)
        return b;
    else return a;
}
// define the height calculation
float calculateHeight(int row, int col, float h, float step, int hps_row) {
 
	float result;
    float temp;
 
    if(row <= hps_row/2 && col <= COLS/2){
        temp = step * min( row, col);
    } else if (row > hps_row/2 && col <= COLS/2){
        temp = step * min(hps_row -1 - row, col);
    }else if (row <= hps_row/2 && col > COLS/2){
        temp = step * min( row, COLS -1 - col);
    }else if (row > hps_row/2 && col > COLS/2){
        temp = step * min( hps_row -1 - row, COLS -1 - col);
    }
    result = temp;
    return result;
}

int main(void)
{
	// // === get FPGA addresses ==================
    // // Open /dev/mem
	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) 	{
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}
     
    // get virtual addr that maps to physical
	h2p_lw_virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );	
     
	if( h2p_lw_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap1() failed...\n" );
		close( fd );
		return(1);
	}
     
	//get the pointer
    pio_row = (unsigned int *)(h2p_lw_virtual_base + PIO_ROW);
    pio_col = (unsigned int *)(h2p_lw_virtual_base + PIO_COL);
	pio_data = (int *)(h2p_lw_virtual_base + PIO_DATA);
	pio_rst = (unsigned int *)(h2p_lw_virtual_base + PIO_RST); //in
	pio_rho_gain = (int *)(h2p_lw_virtual_base + PIO_RHOGAIN);
	pio_numrow = (unsigned int *)(h2p_lw_virtual_base + PIO_NUMROW);

	//-------------------- initialize the matrix -------------------------------------------//

	int numrow;
	printf("Please input the num of rows:\n");
	scanf("%d",&numrow);
	*pio_numrow = numrow;

	float strength;
	printf("Please input the strength of the hit in range [0, 0.125]:\n");
	scanf("%f",&strength);

    float arr[numrow][COLS];
    //float step = strength /(((ROWS-1)/2)) ;
	float step = strength /((min(numrow-1,COLS-1)/2.0)) ;
	int i, j;
	
	*pio_row = 0 ;
	*pio_col = 0 ;
	*pio_data = 0;

	float rho_gain;
	printf("Please input the rho_gain in range [0, 0.49]:\n");
	scanf("%f", &rho_gain);
	*pio_rho_gain = float2fix(rho_gain);

	while(1){
		*pio_row = 0 ;
		*pio_col = 0 ;
		*pio_data = 0;
		while(!(*pio_rst)){
			printf("Wait for reset!\n");
		}
		while(*pio_rst){
			printf("Wait for reset release!\n");
		}
		// initialization
		for ( i = 0; i < numrow; ++i) {
			for ( j = 0; j < COLS; ++j) {
				arr[i][j] = calculateHeight(i, j, H, step, numrow); 
				*pio_row = i ;
				*pio_col = j ;
				*pio_data = float2fix(arr[i][j]);
				usleep(0.02);
			}
		}

		// print
		for ( i = 0; i < numrow; ++i) {
			for ( j = 0; j < COLS; ++j) {
				printf("%f ", arr[i][j]);
			}
			printf("\n");
		}
	}
    //-------------------------------------------------------------------------------------------------------------//
	return 0;

} // end main
