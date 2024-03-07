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
#define DX                    0x00000050
#define DY                    0x00000060
#define RESTART               0x00000070
#define RESET                 0x00000080  //this is input from FPGA to HPS
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
volatile signed int   *pio_dx      = NULL;
volatile signed int   *pio_dy      = NULL;
volatile unsigned int *pio_restart = NULL;
volatile unsigned int *pio_reset   = NULL;

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
    pio_end_x   = (int *)(h2p_lw_virtual_base + END_X);
    pio_end_y   = (int *)(h2p_lw_virtual_base + END_Y);
    pio_dx      = (int *)(h2p_lw_virtual_base + DX);
    pio_dy      = (int *)(h2p_lw_virtual_base + DY);
    pio_restart = (int *)(h2p_lw_virtual_base + RESTART);
    pio_reset   = (int *)(h2p_lw_virtual_base + RESET);

    //assign the default value of the maximum iterations
    int usr_max = 1023;
    int restart = 0;
	*pio_max_ite = usr_max;
    *pio_restart = restart;
    
    //set the varibles to store x,y coordinates
    float start_x = float2fix(-2.0);
    float start_y = float2fix(1.0);
    float end_x   = float2fix(1.0);
    float end_y   = float2fix(-1.0);
    float dx      = float2fix(3/640.0);
    float dy      = float2fix(2/480.0);

    *pio_start_x = start_x;
    *pio_start_y = start_y;
    *pio_end_x   = end_x;
    *pio_end_y   = end_y;
    *pio_dx      = dx;
    *pio_dy      = dy;
    

    //set up the maximum iteration number
    printf("Please enter the max number of iterations from 1 - 1023:\n");
    scanf("%d", &usr_max);
        while(usr_max < 1 || usr_max > 1023){
            printf("Outside range, please input again:\n");
            scanf("%d", &usr_max);
        }
            *pio_max_ite = usr_max;
    printf("SET MAX ITERATION NUM!\n");

    printf("DONE_INITIALIZATION!\n");
  
	//-------------------- assign the max number of iterations values -------------------------------------------//
    char zp;
    float input_start_x = -2.0;
    float input_start_y = 1.0;
    float input_end_x   = 1.0;
    float input_end_y   = -1.0;
    float input_dx      = 3/640.0;
    float input_dy      = 2/480.0;
    float input_restart = 0;
    printf("The X-range is [%f, %f]\n",input_start_x, input_end_x);
    printf("The Y-range is [%f, %f]\n",input_start_y, input_end_y);

    while(1){
        if(*pio_reset == 1){
            *pio_start_x = start_x;
            *pio_start_y = start_y;
            *pio_end_x   = end_x;
            *pio_end_y   = end_y;
            *pio_dx      = dx;
            *pio_dy      = dy;
        }else{
            printf("Panning using 'W' 'S' 'A' 'D' keys, zooming using 'N' 'M' keys.\n");
            scanf("%c",&zp);
           // printf("%c, %c\n", zp, 'w');
            //detect the panning buttons
            if(zp == 'w'){
                input_start_x = input_start_x;
                input_start_y = input_start_y + 0.2;
                input_dx      = input_dx;
                input_dy      = input_dy;
                start_x = float2fix(input_start_x);
                start_y = float2fix(input_start_y);
                dx      = float2fix(input_dx);
                dy      = float2fix(input_dy);
                *pio_start_x = start_x;
                *pio_start_y = start_y;
                *pio_dx      = dx;
                *pio_dy      = dy;
                *pio_restart = 1;
                usleep(7000);
                *pio_restart = 0;
                printf("The X-range is [%f, %f]\n",input_start_x, input_start_x + 3.0);
                printf("The Y-range is [%f, %f]\n",input_start_y, input_start_y - 2.0);

            }else if(zp == 's'){
                input_start_x = input_start_x;
                input_start_y = input_start_y - 0.2;
                input_dx      = input_dx;
                input_dy      = input_dy;
                start_x = float2fix(input_start_x);
                start_y = float2fix(input_start_y);
                dx      = float2fix(input_dx);
                dy      = float2fix(input_dy);
                *pio_start_x = start_x;
                *pio_start_y = start_y;
                *pio_dx      = dx;
                *pio_dy      = dy;
                *pio_restart = 1;
                usleep(7000);
                *pio_restart = 0;
                printf("The X-range is [%f, %f]\n",input_start_x, input_start_x + 3.0);
                printf("The Y-range is [%f, %f]\n",input_start_y, input_start_y - 2.0);

                
            }else if(zp == 'a'){
                input_start_x = input_start_x - 0.3;
                input_start_y = input_start_y;
                input_dx      = input_dx;
                input_dy      = input_dy;
                start_x = float2fix(input_start_x);
                start_y = float2fix(input_start_y);
                dx      = float2fix(input_dx);
                dy      = float2fix(input_dy);
                *pio_start_x = start_x;
                *pio_start_y = start_y;
                *pio_dx      = dx;
                *pio_dy      = dy;
                *pio_restart = 1;
                usleep(7000);
                *pio_restart = 0;
                printf("The X-range is [%f, %f]\n",input_start_x, input_start_x + 3.0);
                printf("The Y-range is [%f, %f]\n",input_start_y, input_start_y - 2.0);


            }else if(zp == 'd'){
                input_start_x = input_start_x + 0.3;
                input_start_y = input_start_y;
                input_dx      = input_dx;
                input_dy      = input_dy;
                start_x = float2fix(input_start_x);
                start_y = float2fix(input_start_y);
                dx      = float2fix(input_dx);
                dy      = float2fix(input_dy);
                *pio_start_x = start_x;
                *pio_start_y = start_y;
                *pio_dx      = dx;
                *pio_dy      = dy;
                *pio_restart = 1;
                usleep(7000);
                *pio_restart = 0;
                printf("The X-range is [%f, %f]\n",input_start_x, input_start_x + 3.0);
                printf("The Y-range is [%f, %f]\n",input_start_y, input_start_y - 2.0);

            }else if(zp == 'n'){
                input_start_x = input_start_x;
                input_start_y = input_start_y;
                input_dx      = input_dx * 1.2;
                input_dy      = input_dy * 1.2;
                start_x = float2fix(input_start_x);
                start_y = float2fix(input_start_y);
                dx      = float2fix(input_dx);
                dy      = float2fix(input_dy);
                *pio_start_x = start_x;
                *pio_start_y = start_y;
                *pio_dx      = dx;
                *pio_dy      = dy;
                *pio_restart = 1;
                usleep(7000);
                *pio_restart = 0;
                printf("The X-range is [%f, %f]\n",input_start_x, input_start_x + 3.0);
                printf("The Y-range is [%f, %f]\n",input_start_y, input_start_y - 2.0);
            }else if(zp == 'm'){
                input_start_x = input_start_x;
                input_start_y = input_start_y;
                input_dx      = input_dx / 1.2;
                input_dy      = input_dy / 1.2;
                start_x = float2fix(input_start_x);
                start_y = float2fix(input_start_y);
                dx      = float2fix(input_dx);
                dy      = float2fix(input_dy);
                *pio_start_x = start_x;
                *pio_start_y = start_y;
                *pio_dx      = dx;
                *pio_dy      = dy;
                *pio_restart = 1;
                usleep(7000);
                *pio_restart = 0;
                printf("The X-range is [%f, %f]\n",input_start_x, input_start_x + 3.0);
                printf("The Y-range is [%f, %f]\n",input_start_y, input_start_y - 2.0);
            }else{
                //refresh the current state
                input_start_x = input_start_x;
                input_start_y = input_start_y;
                input_dx      = input_dx;
                input_dy      = input_dy;
                start_x = float2fix(input_start_x);
                start_y = float2fix(input_start_y);
                dx      = float2fix(input_dx);
                dy      = float2fix(input_dy);
                *pio_start_x = start_x;
                *pio_start_y = start_y;
                *pio_dx      = dx;
                *pio_dy      = dy;
                *pio_restart = 1;
                usleep(7000);
                *pio_restart = 0;
                printf("The X-range is [%f, %f]\n",input_start_x, input_start_x + 3.0);
                printf("The Y-range is [%f, %f]\n",input_start_y, input_start_y - 2.0);
            }
            
        }
       
    }
    
	return 0;

} // end main
