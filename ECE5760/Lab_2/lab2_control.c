///////////////////////////////////////
/// 640x480 version! 16-bit color
/// This code will segfault the original
/// DE1 computer
/// compile with
/// gcc lab2_control.c -o gr -O2 -lm
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

//#include "address_map_arm_brl4.h"

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
#define PIO_MAX_ITER          0x00000000
#define START_X               0x00000010
#define START_Y               0x00000020
#define END_X                 0x00000030
#define END_Y                 0x00000040
//------------------------------------------------------------//

// the light weight buss base
void *h2p_lw_virtual_base;

// /dev/mem file id
int fd;

// define the data converter for fix point 4.23
typedef signed int fix;
#define int2fix(x)   (fix) (x <<23);
#define float2fix(x) (fix)(x * 8388608.0);

#define fix2float(x) ((float)(x) / 8388608.0);
#define fix2int(x)   (int) (x >> 23);

// === need to mmap: =======================
// FPGA_CHAR_BASE
// FPGA_ONCHIP_BASE      
// HW_REGS_BASE        
volatile unsigned int *pio_max_ite = NULL;
volatile signed int   *pio_start_x = NULL;
volatile signed int   *pio_start_y = NULL;
volatile signed int   *pio_end_x   = NULL;
volatile signed int   *pio_end_y   = NULL;

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
    pio_max_ite = (int *)(h2p_lw_virtual_base + PIO_MAX_ITER);
    pio_start_x = (int *)(h2p_lw_virtual_base + START_X);
    pio_start_y = (int *)(h2p_lw_virtual_base + START_Y);
    pio_end_x = (int *)(h2p_lw_virtual_base + END_X);
    pio_end_y = (int *)(h2p_lw_virtual_base + END_Y);

    //assign the default value of the maximum iterations
    int usr_max = 1023;
	
    //set the varibles to store x,y coordinates
    float start_x = fix2float(*pio_start_x);
    float start_y = fix2float(*pio_start_y);
    float end_x   = fix2float(*pio_end_x);
    float end_y   = fix2float(*pio_end_y);
	//-------------------- assign the max number of iterations values -------------------------------------------//
    while(1){
        printf("Please enter the max number of iterations from 1 - 1023:\n");
        scanf("%d", &usr_max);
        while(usr_max < 1 || usr_max > 1023){
            printf("Outside range, please input again:\n");
            scanf("%d", &usr_max);
        }
            *pio_max_ite = usr_max;
            printf("DONE!\n");
        printf("The X-range is [%f, %f]\n",start_x, end_x);
        printf("The Y-range is [%f, %f]\n",start_y, end_y);
       
    }
	return 0;

} // end main
