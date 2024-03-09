///////////////////////////////////////
/// Audio
/// compile with
/// gcc media_brl4_7_audio_drum.c -o testA -lm -O3
/// works up to about drum size 30 or so for NO multiplies case
/// 
///////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <sys/types.h>
#include <string.h>
// interprocess comm
#include <sys/ipc.h> 
#include <sys/shm.h> 
#include <sys/mman.h>
#include <time.h>
// network stuff
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include "address_map_arm_brl4.h"

// fixed point
#define float2fix30(a) ((int)((a)*1073741824)) // 2^30

#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

/* function prototypes */
void VGA_text (int, int, char *);
void VGA_box (int, int, int, int, short);
void VGA_line(int, int, int, int, short) ;

// virtual to real address pointers

volatile unsigned int * red_LED_ptr = NULL ;
//volatile unsigned int * res_reg_ptr = NULL ;
//volatile unsigned int * stat_reg_ptr = NULL ;

// audio stuff
volatile unsigned int * audio_base_ptr = NULL ;
volatile unsigned int * audio_fifo_data_ptr = NULL ; //4bytes
volatile unsigned int * audio_left_data_ptr = NULL ; //8bytes
volatile unsigned int * audio_right_data_ptr = NULL ; //12bytes
// phase accumulator
// drum-specific multiply macros simulated by shifts
#define times0pt5(a) ((a)>>1) 
#define times0pt25(a) ((a)>>2) 
#define times2pt0(a) ((a)<<1) 
#define times4pt0(a) ((a)<<2) 
#define times0pt9998(a) ((a)-((a)>>12)) //>>10
#define times0pt9999(a) ((a)-((a)>>13)) //>>10
#define times0pt999(a) ((a)-((a)>>10)) //>>10
#define times_rho(a) (((a)>>5)) //>>2

float max(float a, float b){
	return (a > b) ? a : b;
}

float min(float a, float b){
	return (a > b) ? b : a;
}

// drum size paramenters
// drum will FAIL if size is too big
// #define drum_size 30
// #define drum_middle 15
#define drum_size 100 //update the drum size!
#define drum_middle 50
int copy_size = drum_size*drum_size*4 ;

// fixed pt macros suitable for 32-bit sound
typedef signed int fix28 ;
//multiply two fixed 4:28
#define multfix28(a,b) ((fix28)(((( signed long long)(a))*(( signed long long)(b)))>>28)) 
//#define multfix28(a,b) ((fix28)((( ((short)((a)>>17)) * ((short)((b)>>17)) )))) 
#define float2fix28(a) ((fix28)((a)*268435456.0f)) // 2^28
#define fix2float28(a) ((float)(a)/268435456.0f) 
#define int2fix28(a) ((a)<<28)
#define fix2int28(a) ((a)>>28)
// shift fraction to 32-bit sound
#define fix2audio28(a) (a<<4)
// shift fraction to 16-bit sound
#define fix2audio16(a) (a>>12)

// some fixed point values
#define FOURfix28 0x40000000 
#define SIXTEENTHfix28 0x01000000
#define ONEfix28 0x10000000

//-------------------------------initializize drum----------------------------//

// #define drum_size 100
#define rho 0.25
fix28 rho_eff = 0.25;
#define Fs 48000
#define LENGTH 100

// float time_t[LENGTH];
fix28 u[LENGTH][LENGTH];
fix28 u1[LENGTH][LENGTH];
fix28 u2[LENGTH][LENGTH];
fix28 uHit[LENGTH][LENGTH];

int t;

// drum state variable arrays
fix28 drum_n[drum_size][drum_size] ;
// drup amp at last time
fix28 drum_n_1[drum_size][drum_size] ;
// drum updata
fix28 new_drum[drum_size][drum_size] ;
fix28 new_drum_temp ;

clock_t note_time ;

// the light weight buss base
void *h2p_lw_virtual_base;

// pixel buffer
volatile unsigned int * vga_pixel_ptr = NULL ;
void *vga_pixel_virtual_base;

// character buffer
volatile unsigned int * vga_char_ptr = NULL ;
void *vga_char_virtual_base;

// /dev/mem file descriptor
int fd;

// shared memory 
key_t mem_key=0xf0;
int shared_mem_id; 
int *shared_ptr;
int audio_time;

// width of gaussian initial condition
float alpha = 64;
 
int main(void)
{
	// Declare volatile pointers to I/O registers (volatile 	// means that IO load and store instructions will be used 	// to access these pointer locations, 
	// instead of regular memory loads and stores) 

  	// === shared memory =======================
	// with video process
	shared_mem_id = shmget(mem_key, 100, IPC_CREAT | 0666);
	shared_ptr = shmat(shared_mem_id, NULL, 0);
	
	// === need to mmap: =======================
	// FPGA_CHAR_BASE
	// FPGA_ONCHIP_BASE      
	// HW_REGS_BASE        
  
	// === get FPGA addresses ==================
    // Open /dev/mem
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
    
    // Get the address that maps to the FPGA LED control 
	red_LED_ptr =(unsigned int *)(h2p_lw_virtual_base +  	 			LEDR_BASE);

	// address to resolution register
	//res_reg_ptr =(unsigned int *)(h2p_lw_virtual_base +  	 	//		resOffset);

	 //addr to vga status
	//stat_reg_ptr = (unsigned int *)(h2p_lw_virtual_base +  	 	//		statusOffset);

	// audio addresses
	// base address is control register
	audio_base_ptr = (unsigned int *)(h2p_lw_virtual_base +  	 			AUDIO_BASE);
	audio_fifo_data_ptr  = audio_base_ptr  + 1 ; // word
	audio_left_data_ptr = audio_base_ptr  + 2 ; // words
	audio_right_data_ptr = audio_base_ptr  + 3 ; // words

	// === get VGA char addr =====================
	// get virtual addr that maps to physical
	vga_char_virtual_base = mmap( NULL, FPGA_CHAR_SPAN, ( 	PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_CHAR_BASE );	
	if( vga_char_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap2() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the FPGA LED control 
	vga_char_ptr =(unsigned int *)(vga_char_virtual_base);

	// === get VGA pixel addr ====================
	// get virtual addr that maps to physical
	vga_pixel_virtual_base = mmap( NULL, FPGA_ONCHIP_SPAN, ( 	PROT_READ | PROT_WRITE ), MAP_SHARED, fd, 			FPGA_ONCHIP_BASE);	
	if( vga_pixel_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap3() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the FPGA pixel buffer
	vga_pixel_ptr =(unsigned int *)(vga_pixel_virtual_base);

	// ===========================================
	// drum index
	int i, j, dist2;
		
	// read the LINUX clock (microSec)
	// and set the time so that a not plays soon
	note_time = clock() - 2800000;

	//------------drum------------------------//
	// for (int i = 0; i < LENGTH; i++){
	// 	time_t[i] = (float) i/Fs;
	// }
	int p = 0, q = 0;
	for (p = 0; p < LENGTH; p++){
		for ( q = 0; q < LENGTH; q++) {
			u[p][q] = 0.0;
			u1[p][q] = 0.0;
			u2[p][q] = 0.0;
			uHit[p][q]= 0.0;
		}
		
	}
	int x_mid = drum_middle;
	int y_mid = drum_middle;

	int pi, pj;
	for( pi = 0; pi < LENGTH; pi++){
		for ( pj = 0 ; pj < LENGTH; pj++){
			if (pi == 0 || pj == 0 || pj == LENGTH-1 || pi == LENGTH-1) {
				uHit[pi][pj] = 0;
			} else {
				uHit[pi][pj] = float2fix28(max(0.0 , 30-(abs(x_mid - pi) + abs(y_mid - pj)) ) /30);
			}
		}
	}
	
	

	while(1){	

		// generate a drum simulation
		// load the FIFO until it is full
		while (((*audio_fifo_data_ptr>>24)& 0xff) > 1) {
			// do drum time sample
			// equation 2.18 
			// from http://people.ece.cornell.edu/land/courses/ece5760/LABS/s2018/WaveFDsoln.pdf
			// for (i=1; i<drum_size-1; i++){
			// 	for (j=1; j<drum_size-1; j++){
			// 		new_drum_temp = times_rho(drum_n[i-1][j] + drum_n[i+1][j] + drum_n[i][j-1] + drum_n[i][j+1] - times4pt0(drum_n[i][j]));
			// 		new_drum[i][j] = times0pt9999(new_drum_temp + times2pt0(drum_n[i][j]) - times0pt9998(drum_n_1[i][j])) ;
			// 	}
			// }

			// for t in time: python;
			int time_i = 0;
			printf("Inside FIFO, FIFO is not full\n");

			for ( time_i = 0; time_i < Fs; time_i++){
				//printf("Inside Time, Time is not full\n");
				int pt = 0, qt = 0;
				for (pt = 0; pt < LENGTH; pt++){
					for ( qt = 0; qt < LENGTH; qt++){
						u[pt][qt] = 0.0;
						// u1[pt][qt] = (float)0.0;
						// u2[pt][qt] = (float)0.0;
						// uHit[pt][qt]= (float)0.0;
					}
				}
				int ii,ji;
				if (time_i == 0){
					for ( ii = 0; ii < LENGTH; ii++){
						for ( ji = 0; ji < LENGTH; ji++){
							u1[ii][ji] = u1[ii][ji] + uHit[ii][ji];
						}
					}
				}
		
				int t, ti;
				for( t = 1; t < LENGTH-1; t++ ){
					for( ti = 1; ti < LENGTH-1; ti++){
						u[t][ti] = float2fix28((1.0/(1.0 + 0.0001)) * (rho_eff * (u1[t + 1][ti] + u1[t-1][ti] + u1[t][ti + 1]+u1[t][ti-1] - times4pt0 (u1[t][ti]) + times2pt0(u1[t][ti]) - (1-0.0001) * u2[t][ti])) );
						printf("result of u[%d][%d], %f \n",t,ti,u[t][ti]);
					}
				}


				memcpy((void*)u2, (void*)u1, copy_size);
				memcpy((void*)u1, (void*)u, copy_size);
				//printf("First data, %f \n", u[drum_middle][drum_middle]);
				// send time sample to the audio FiFOs
				*audio_left_data_ptr = fix2audio16((int)u[drum_middle][drum_middle]);
				*audio_right_data_ptr = fix2audio16((int)u[drum_middle][drum_middle]);
				printf("Inside FIFO, FIFO is not full\n");

				rho_eff = min ((float)0.49, (float) (rho + ((1.0 / 16) * u[drum_middle][drum_middle]) * ((1.0 / 16) * u[drum_middle][drum_middle])) );
				

			}
			
			// update the state arrays
			// memcpy((void*)u2, (void*)u1, copy_size);
			// memcpy((void*)u1, (void*)u, copy_size);
			
			// // send time sample to the audio FiFOs
			// *audio_left_data_ptr = fix2audio16(u[drum_middle][drum_middle]);
			// *audio_right_data_ptr = fix2audio16(u[drum_middle][drum_middle]);

			// rho_eff = min (0.49, rho + ((1.0 / 16) * u[drum_middle][drum_middle]) * ((1.0 / 16) * u[drum_middle][drum_middle]));
			// shared memory for possible graphics
			//*(shared_ptr+1) = fix2audio28(drum_n[drum_middle][drum_middle]);
			// share the audio sample time with video process
			//audio_time++ ;
			//*shared_ptr = audio_time/48000 ;
		} // end while (((*audio	
		
		if (clock()- note_time > 3000000) {
			// strike the drum
			// this is a set up for zerro initial displacment with
			// the strike as a velocity
			for (i=1; i<drum_size-1; i++){
				for (j=1; j<drum_size-1; j++){
					dist2 = (i-drum_middle)*(i-drum_middle)+(j-drum_middle)*(j-drum_middle);
					u2[i][j] = float2fix28(0.01*exp(-(float)dist2/alpha));
					u[i][j] = 0 ;
				}
			}
			
			//read LINUX time for the next drum strike
			note_time = clock();
			
		};

	} // end while(1)
} // end main
