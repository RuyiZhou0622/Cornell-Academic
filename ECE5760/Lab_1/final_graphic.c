///////////////////////////////////////
/// 640x480 version! 16-bit color
/// This code will segfault the original
/// DE1 computer
/// compile with
/// gcc graphics_video_16bit.c -o gr -O2 -lm
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
#define PIO_NEW_X             0x00000010
#define PIO_NEW_Y             0x00000020
#define PIO_NEW_Z             0x00000030

#define PIO_INITIAL_X         0x00000040
#define PIO_INITIAL_Y         0x00000050
#define PIO_INITIAL_Z         0x00000060

#define PIO_RHO               0x00000070
#define PIO_SIGMA             0x00000080
#define PIO_BETA              0x00000090

#define PIO_DT                0x000000A0
#define PIO_CLK               0x000000B0
#define PIO_RST               0x000000C0
//------------------------------------------------------------//

// graphics primitives
void VGA_text (int, int, char *);
void VGA_text_clear();
void VGA_box (int, int, int, int, short);
void VGA_rect (int, int, int, int, short);
void VGA_line(int, int, int, int, short) ;
void VGA_Vline(int, int, int, short) ;
void VGA_Hline(int, int, int, short) ;
void VGA_disc (int, int, int, short);
void VGA_circle (int, int, int, int);
void VGA_Sine(short);
void *read1();
void *write1();
void *counter1();
// 16-bit primary colors
#define red  (0+(0<<5)+(31<<11))
#define dark_red (0+(0<<5)+(15<<11))
#define green (0+(63<<5)+(0<<11))
#define dark_green (0+(31<<5)+(0<<11))
#define blue (31+(0<<5)+(0<<11))
#define dark_blue (15+(0<<5)+(0<<11))
#define yellow (0+(63<<5)+(31<<11))
#define cyan (31+(63<<5)+(0<<11))
#define magenta (31+(0<<5)+(31<<11))
#define black (0x0000)
#define gray (15+(31<<5)+(51<<11))
#define white (0xffff)

int colors[] = {red, dark_red, green, dark_green, blue, dark_blue, 
		yellow, cyan, magenta, gray, black, white};

// pixel macro
#define VGA_PIXEL(x,y,color) do{\
	int  *pixel_ptr ;\
	pixel_ptr = (int*)((char *)vga_pixel_ptr + (((y)*640+(x))<<1)) ; \
	*(short *)pixel_ptr = (color);\
} while(0)

// the light weight buss base
void *h2p_lw_virtual_base;

// pixel buffer
volatile unsigned int * vga_pixel_ptr = NULL ;
void *vga_pixel_virtual_base;

// character buffer
volatile unsigned int * vga_char_ptr = NULL ;
void *vga_char_virtual_base;

// /dev/mem file id
int fd;

// measure time
struct timeval t1, t2;
double elapsedTime;

typedef signed int fix;
#define int2fix(x)   (fix) (x <<20);
#define float2fix(x) (fix)(x * 1048576.0);

#define fix2float(x) ((float)(x) / 1048576.0);
#define fix2int(x)   (int) (x >> 20);

char input_buffer[64];

// access to enter condition
// -- for signalling enter done
pthread_mutex_t enter_lock= PTHREAD_MUTEX_INITIALIZER;
// access to print condition
// -- for signalling print done
pthread_mutex_t print_lock= PTHREAD_MUTEX_INITIALIZER;
// counter protection
pthread_mutex_t count_lock= PTHREAD_MUTEX_INITIALIZER;

// the two condition variables related to the mutex above
pthread_cond_t enter_cond ;
pthread_cond_t print_cond ;

// globals for perfromance
int count1, count2;
int mode = 1;


//Pthread
int counter = 0;
int reset_signal = 0;
int speedup = 0;
int speeddown = 0;
int edit_signal = 0;
int edit = 0;
int pause_signal = 0;
int clear = 0;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_a = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
pthread_cond_t cond_speedup = PTHREAD_COND_INITIALIZER;
pthread_cond_t cond_speeddown = PTHREAD_COND_INITIALIZER;
pthread_cond_t cond_pause = PTHREAD_COND_INITIALIZER;
pthread_cond_t cond_edit = PTHREAD_COND_INITIALIZER;
pthread_cond_t cond_input = PTHREAD_COND_INITIALIZER;
pthread_cond_t cond_clear = PTHREAD_COND_INITIALIZER;

	double x;
	double y;
	double z;
	double x1;
	double q1;
	double z1;
	double x2;
	double y2;
	double z2;
	double temp_x;
	double temp_y;
	double temp_z;
	double temp_x1;
	double temp_y1;
	double temp_z1;
	double temp_x2;
	double temp_y2;
	double temp_z2;
	int USLEEP_TIME = 5000;
	
	 float ini_x;
	 float ini_y;
	 float ini_z;
	 float ini_rho;
	 float ini_sigma;
	 float ini_beta;
	 float ini_dt;
	// === need to mmap: =======================
	// FPGA_CHAR_BASE
	// FPGA_ONCHIP_BASE      
	// HW_REGS_BASE        
   	 volatile signed int *new_x=NULL;
     volatile signed int *new_y=NULL;
	 volatile signed int *new_z=NULL;

	 volatile signed int *initial_x=NULL;
     volatile signed int *initial_y=NULL;
	 volatile signed int *initial_z=NULL;

	 volatile signed int *rho   = NULL;
	 volatile signed int *sigma = NULL;
	 volatile signed int *beta  = NULL;

	 volatile signed int *dt      = NULL;
	 volatile char *sys_clk = NULL;    //////
	 volatile char *sys_rst = NULL;    ////////
	 char text_top_row[40] = "DE1-SoC ARM/FPGA\0";
	 char text_bottom_row[40] = "Cornell ece5760\0";
	 char text_next[40] = "Graphics primitives\0";


	char num_string[20], time_string[20] ;
	char color_index = 0 ;
	int color_counter = 0 ;
	
	// position of disk primitive
	int disc_x = 0;
	// position of circle primitive
	int circle_x = 0 ;
	// position of box primitive
	int box_x = 5 ;
	// position of vertical line primitive
	int Vline_x = 350;
	// position of horizontal line primitive
	int Hline_y = 250;



//edit

void *threadmain(void *args){
	while(1) {	
		char input;
		//pthread
		pthread_mutex_lock(&mutex);
		
		while (pause_signal) {
			pthread_cond_wait(&cond_pause, &mutex);
		}

		if(reset_signal) {
			// reset_signal = 0;
			
			// temp_x = (-1048576 / 1048576.0)*3 + 150;
			// temp_y = (104857 / 1048576.0)*3 + 150;

			// temp_x1 = (-1048576 / 1048576.0)*3 + 300;
			// temp_z1 = (26214400 / 1048576.0)*3 + 300;

			// temp_y2 = (104857 / 1048576.0)*3 + 500;
			// temp_z2 = (26214400 / 1048576.0)*3 + 150;

            // printf("reset!\n");


			// *sys_clk = 0;
			// *sys_rst = 1;
			// usleep(USLEEP_TIME);

			// *sys_clk = 1;
			// *sys_rst = 0;
			// usleep(USLEEP_TIME);

			// *sys_clk = 0;
			// usleep(USLEEP_TIME);

			// VGA_box (0, 0, 639, 479, 0x0000);
			// VGA_text_clear();
			// VGA_box (0, 0, 639, 479, 0x0000);

			//pthread_cond_signal(&cond);
		} else if (speedup) {
			//printf("speedup");
			//USLEEP_TIME = 1000;
            //USLEEP_TIME -= 1000;
            // if(USLEEP_TIME > 1000){
            //     USLEEP_TIME -= 1000;
            //     printf("speedup is set! USLEEP_TIME = %d\n", USLEEP_TIME);
            // }else{
            //     printf("reach limited!\n");
            // }
			// speedup = 0;
		} else if (speeddown) {
	
			// USLEEP_TIME += 1000;
            // if(USLEEP_TIME == 0){
            //     USLEEP_TIME == 500;// thread_clear
            // }
			// speeddown = 0;
			// printf("speeddown is set! USLEEP_TIME = %d\n", USLEEP_TIME);

		} else if (pause_signal) {
			// pause_signal = 0;
			printf("pause now!\n ");
		} else if (clear) {
            VGA_box (0, 0, 639, 479, 0x0000);
			//VGA_text_clear();
			//VGA_box (0, 0, 639, 479, 0x0000);
            clear = 0;
            printf("clear already!\n");
        }
        else {
			char text_rho[40];
			char text_sigma[40];
			char text_beta[40];

			sprintf(text_rho, "The rho is: %.5f ", ini_rho);
			sprintf(text_sigma, "The sigma is: %.5f ", ini_sigma);
			sprintf(text_beta, "The beta is: %.5f ", ini_beta);

			VGA_text (34, 1, text_rho);
			VGA_text (34, 2, text_sigma);
			VGA_text (34, 3, text_beta);
			
			*sys_clk = 1;
			usleep(USLEEP_TIME);
			*sys_clk = 0;		

			x =  ((float)(*new_x) / 1048576.0) * 3 + 150;
			y =  ((float)(*new_y) / 1048576.0) * 3 + 150;

			x1 = ((float)(*new_x) / 1048576.0) * 3 + 300;
			z1 = ((float)(*new_z) / 1048576.0) * 3 + 300;

			y2 = ((float)(*new_y) / 1048576.0) * 3 + 500;
			z2 = ((float)(*new_z) / 1048576.0) * 3 + 150;

			//	printf("The x is:%d   %f\n",*new_x, x);
			//	printf("The y is:%d   %f\n",*new_y, y);
			//	printf("The z is:%d   %f\n",*new_z, z);
			//	printf("The rst is:%d\n",*sys_rst);		
			// start timer
			// gettimeofday(&t1, NULL);
		
			VGA_line(temp_x, temp_y, x, y, blue);
			VGA_line(temp_x1, temp_z1, x1, z1, green);
			VGA_line(temp_y2, temp_z2, y2, z2, red);

			
			temp_x = x;
			temp_y = y;
			temp_z = z;

			temp_x1 = x1;
			temp_y1 = q1;
			temp_z1 = z1;

			temp_x2 = x2;
			temp_y2 = y2;
			temp_z2 = z2;
		}
		
		// stop timer
		gettimeofday(&t2, NULL);
		elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000000.0;      // sec to us
		elapsedTime += (t2.tv_usec - t1.tv_usec) ;   // us 

		pthread_mutex_unlock(&mutex);
		// set frame rate
		usleep(USLEEP_TIME);
		

	} // end while(1)
	return NULL;
}
////

void *reset_thread(void *args){
	while(1){
		pthread_mutex_lock(&mutex);
		while(!reset_signal){
			pthread_cond_wait(&cond, &mutex);
            reset_signal = 0;
			
			temp_x = (-1048576 / 1048576.0)*3 + 150;
			temp_y = (104857 / 1048576.0)*3 + 150;

			temp_x1 = (-1048576 / 1048576.0)*3 + 300;
			temp_z1 = (26214400 / 1048576.0)*3 + 300;

			temp_y2 = (104857 / 1048576.0)*3 + 500;
			temp_z2 = (26214400 / 1048576.0)*3 + 150;

            printf("reset!\n");


			*sys_clk = 0;
			*sys_rst = 1;
			usleep(USLEEP_TIME);

			*sys_clk = 1;
			*sys_rst = 0;
			usleep(USLEEP_TIME);

			*sys_clk = 0;
			usleep(USLEEP_TIME);

			VGA_box (0, 0, 639, 479, 0x0000);
			VGA_text_clear();
			VGA_box (0, 0, 639, 479, 0x0000);

		}
		pthread_mutex_unlock(&mutex);
	}
	return NULL;
}

void *speedup_thread(void *args){
	while(1){
		pthread_mutex_lock(&mutex);
		//printf("this is speedup");
		while(!speedup){
			pthread_cond_wait(&cond_speedup, &mutex);
            if(USLEEP_TIME > 1000){
                USLEEP_TIME -= 1000;
                printf("speedup is set! USLEEP_TIME = %d\n", USLEEP_TIME);
            }else{
                printf("reach limited!\n");
            }
			speedup = 0;
		}
		pthread_mutex_unlock(&mutex);
	}
}

void *speeddown_thread(void *args){
	while(1){
		pthread_mutex_lock(&mutex);
		while(!speeddown){
			pthread_cond_wait(&cond_speeddown, &mutex);
            USLEEP_TIME += 1000;
            if(USLEEP_TIME == 0){
                USLEEP_TIME == 500;// thread_clear
            }
			speeddown = 0;
			printf("speeddown is set! USLEEP_TIME = %d\n", USLEEP_TIME);
		}
		pthread_mutex_unlock(&mutex);
	}
}

void *pause_thread(void *args){
	while(1){
		pthread_mutex_lock(&mutex);
		//printf("this is pause");
		while(!pause_signal){
			pthread_cond_wait(&cond_pause, &mutex);
		}
		pthread_mutex_unlock(&mutex);
	}
	return NULL;
}

void *clear_thread(void *args){
	while(1){
		pthread_mutex_lock(&mutex);
		//printf("this is pause");
		while(!clear){
			pthread_cond_wait(&cond_clear, &mutex);
		}
		pthread_mutex_unlock(&mutex);
	}
	return NULL;
}


void *edit_thread(void *args){
	while(1){
        //printf("this is edit thread. edit = %d\n",edit);
        sleep(1);
		if(edit == 1){
            sleep(0.2);

			printf("Please enter the rho:\n");
			scanf("%f", &ini_rho);
			printf("Please enter the sigma:\n");
			scanf("%f", &ini_sigma);
			printf("Please enter the beta:\n");
			scanf("%f", &ini_beta);

			printf("rho sigma beta change to %f, %f, %f:\n",ini_rho, ini_sigma, ini_beta);
			// scanf("%f", &ini_dt);
            *rho   = float2fix (ini_rho);
            *sigma = float2fix (ini_sigma);
            *beta  = float2fix (ini_beta);

			edit = 0;
		}
	}
	return NULL;
}


int main(void)
{
	
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
    
	//----------------get the address the maps to the ports-------------------------------------//
	new_x = (signed int *)(h2p_lw_virtual_base + PIO_NEW_X);
	new_y = (signed int *)(h2p_lw_virtual_base + PIO_NEW_Y);
	new_z = (signed int *)(h2p_lw_virtual_base + PIO_NEW_Z);

	initial_x = (signed int *)(h2p_lw_virtual_base + PIO_INITIAL_X);
	initial_y = (signed int *)(h2p_lw_virtual_base + PIO_INITIAL_Y);
	initial_z = (signed int *)(h2p_lw_virtual_base + PIO_INITIAL_Z);

	rho   =  (signed int *)(h2p_lw_virtual_base + PIO_RHO);
	sigma =  (signed int *)(h2p_lw_virtual_base + PIO_SIGMA);
	beta  =  (signed int *)(h2p_lw_virtual_base + PIO_BETA);

	dt      =  (signed int *)(h2p_lw_virtual_base + PIO_DT);
	sys_clk =  (unsigned char *)(h2p_lw_virtual_base + PIO_CLK);
	sys_rst =  (unsigned char *)(h2p_lw_virtual_base + PIO_RST);
	//------------------------------------------------------------------------------------------//
	
	//-------------------- assign the initial values -------------------------------------------//
	printf("Please select the auto mode or user mode:\n");
	printf("1 - Auto            2 - User\n");
	scanf("%d", & mode);
	if(mode == 2){
	printf("Please enter the initial X:\n");
	scanf("%f", &ini_x);
	printf("Please enter the initial Y:\n");
	scanf("%f", &ini_y);
	printf("Please enter the initial Z:\n");
	scanf("%f", &ini_z);

	printf("Please enter the rho:\n");
	scanf("%f", &ini_rho);
	printf("Please enter the sigma:\n");
	scanf("%f", &ini_sigma);
	printf("Please enter the beta:\n");
	scanf("%f", &ini_beta);

	printf("Please enter the dt:\n");
	scanf("%f", &ini_dt);
	}else{
		ini_x = -1;
		ini_y = 0.1;
		ini_z = 25;
		ini_rho = 28;
		ini_sigma = 10;
		ini_beta = 8.0/3;
		ini_dt = 1.0/256;
	}

	*initial_x = float2fix (ini_x);
	*initial_y = float2fix (ini_y);
	*initial_z = float2fix (ini_z);

	*rho   = float2fix (ini_rho);
	*sigma = float2fix (ini_sigma);
	*beta  = float2fix (ini_beta);

	*dt = float2fix(ini_dt);

	*sys_clk = 0;
	*sys_rst = 1;
	usleep(USLEEP_TIME);

	*sys_clk = 1;
	*sys_rst = 0;
	usleep(USLEEP_TIME);

	*sys_clk = 0;
	usleep(USLEEP_TIME);

	//------------------------------------------------------------------------------------------//
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
	vga_pixel_virtual_base = mmap( NULL, SDRAM_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, SDRAM_BASE);	
	if( vga_pixel_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap3() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the FPGA pixel buffer
	vga_pixel_ptr =(unsigned int *)(vga_pixel_virtual_base);

	// ===========================================

	/* create a message to be displayed on the VGA 
          and LCD displays */
	char text_top_row[40] = "DE1-SoC ARM/FPGA\0";
	char text_bottom_row[40] = "Cornell ece5760\0";
	char text_next[40] = "Graphics primitives\0";


	char num_string[20], time_string[20] ;
	char color_index = 0 ;
	int color_counter = 0 ;
	
	// position of disk primitive
	int disc_x = 0;
	// position of circle primitive
	int circle_x = 0 ;
	// position of box primitive
	int box_x = 5 ;
	// position of vertical line primitive
	int Vline_x = 350;
	// position of horizontal line primitive
	int Hline_y = 250;
    edit = 0;

	// clear the screen
	VGA_box (0, 0, 639, 479, 0x0000);
	// clear the text
	VGA_text_clear();

	VGA_box (0, 0, 639, 479, 0x0000);
	int i = 0;
	temp_x = (-1048576 / 1048576.0)*3 + 150;
	temp_y = (104857 / 1048576.0)*3 + 150;

	temp_x1 = (-1048576 / 1048576.0)*3 + 300;
	temp_z1 = (26214400 / 1048576.0)*3 + 300;

	temp_y2 = (104857 / 1048576.0)*3 + 500;
	temp_z2 = (26214400 / 1048576.0)*3 + 150;

		// the thread identifiers
		pthread_t thread_main, reset_thread_main, thread_up, thread_down, thread_clear, thread_pause, thread_edit;
		
		//pthread_t thread_input;
		// now the threads
		pthread_create(&thread_main, NULL, threadmain, NULL);
		pthread_create(&reset_thread_main, NULL, reset_thread, NULL);
		pthread_create(&thread_down, NULL, speeddown_thread, NULL);
		pthread_create(&thread_up, NULL, speedup_thread, NULL);
		pthread_create(&thread_pause, NULL, pause_thread, NULL);
		pthread_create(&thread_edit, NULL, edit_thread, NULL);
		pthread_create(&thread_clear, NULL, clear_thread, NULL);

		//pthread_create(&thread_input, NULL, thread_input, NULL);

		char input;
            while(1){
                if(edit == 0){
                    //printf("please input value main thread\n");
                    scanf("%c", &input);
                    if (input == 'r'){
                        //printf("main input r");
                        pthread_mutex_lock(&mutex);
                        reset_signal = 1;
                        pthread_cond_signal(&cond);
                        pthread_mutex_unlock(&mutex);
                    } 
                    if (input == '+'){
                        //printf("main input q");
                        pthread_mutex_lock(&mutex);
                        speedup = 1;
                        pthread_cond_signal(&cond_speedup);
                        pthread_mutex_unlock(&mutex);
                    } 
                    if (input == '-'){
                        pthread_mutex_lock(&mutex);
                        speeddown = 1;
                        pthread_cond_signal(&cond_speeddown);
                        pthread_mutex_unlock(&mutex);
                    } if (input == 'c'){
                        pthread_mutex_lock(&mutex);
                        clear = 1;
                        pthread_cond_signal(&cond_clear);
                        pthread_mutex_unlock(&mutex);
                    }  if (input == 'p'){
                        pthread_mutex_lock(&mutex);
                        pause_signal = 1;
                        pthread_cond_signal(&cond_pause);
                        pthread_mutex_unlock(&mutex);
                    }  if(input == 't'){ //turn back
                        pthread_mutex_lock(&mutex);
                        //resume_signal = 1;
                        pause_signal = 0;
                        pthread_cond_signal(&cond_pause);
                        pthread_mutex_unlock(&mutex);
                    }
                    if (input == 'e'){
                        edit = 1;
                        
                        //printf("edit == 1 main process");
                        //printf("main thread edit == %d\n", edit);
                    }

                } else{
                    //printf("main thread : edit == 1/n");
                     
                    sleep(2); }
		    }
        
	// second copy of counter
	//pthread_create(&thread_count2,NULL,counter1,NULL);
		// thread_up, thread_down, thread_pause, thread_edit
	pthread_join(threadmain,NULL);
	pthread_join(reset_thread,NULL);
    pthread_join(thread_up,NULL);
    pthread_join(thread_down,NULL);
    pthread_join(thread_pause,NULL);
    pthread_join(thread_edit,NULL);

	return 0;

} // end main

/****************************************************************************************
 * Subroutine to send a string of text to the VGA monitor 
****************************************************************************************/
void VGA_text(int x, int y, char * text_ptr)
{
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer
	int offset;
	/* assume that the text string fits on one line */
	offset = (y << 7) + x;
	while ( *(text_ptr) )
	{
		// write to the character buffer
		*(character_buffer + offset) = *(text_ptr);	
		++text_ptr;
		++offset;
	}
}

/****************************************************************************************
 * Subroutine to clear text to the VGA monitor 
****************************************************************************************/
void VGA_text_clear()
{
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer
	int offset, x, y;
	for (x=0; x<79; x++){
		for (y=0; y<59; y++){
	/* assume that the text string fits on one line */
			offset = (y << 7) + x;
			// write to the character buffer
			*(character_buffer + offset) = ' ';		
		}
	}
}

/****************************************************************************************
 * Draw a filled rectangle on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_box(int x1, int y1, int x2, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
	if (x1>x2) SWAP(x1,x2);
	if (y1>y2) SWAP(y1,y2);
	for (row = y1; row <= y2; row++)
		for (col = x1; col <= x2; ++col)
		{
			//640x480
			//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
			// set pixel color
			//*(char *)pixel_ptr = pixel_color;	
			VGA_PIXEL(col,row,pixel_color);	
		}
}

/****************************************************************************************
 * Draw a outline rectangle on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_rect(int x1, int y1, int x2, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
	if (x1>x2) SWAP(x1,x2);
	if (y1>y2) SWAP(y1,y2);
	// left edge
	col = x1;
	for (row = y1; row <= y2; row++){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);		
	}
		
	// right edge
	col = x2;
	for (row = y1; row <= y2; row++){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);		
	}
	
	// top edge
	row = y1;
	for (col = x1; col <= x2; ++col){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);
	}
	
	// bottom edge
	row = y2;
	for (col = x1; col <= x2; ++col){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);
	}
}

/****************************************************************************************
 * Draw a horixontal line on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_Hline(int x1, int y1, int x2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (x1>x2) SWAP(x1,x2);
	// line
	row = y1;
	for (col = x1; col <= x2; ++col){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);		
	}
}

/****************************************************************************************
 * Draw a vertical line on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_Vline(int x1, int y1, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (y2<0) y2 = 0;
	if (y1>y2) SWAP(y1,y2);
	// line
	col = x1;
	for (row = y1; row <= y2; row++){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);			
	}
}


/****************************************************************************************
 * Draw a filled circle on the VGA monitor 
****************************************************************************************/

void VGA_disc(int x, int y, int r, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col, rsqr, xc, yc;
	
	rsqr = r*r;
	
	for (yc = -r; yc <= r; yc++)
		for (xc = -r; xc <= r; xc++)
		{
			col = xc;
			row = yc;
			// add the r to make the edge smoother
			if(col*col+row*row <= rsqr+r){
				col += x; // add the center point
				row += y; // add the center point
				//check for valid 640x480
				if (col>639) col = 639;
				if (row>479) row = 479;
				if (col<0) col = 0;
				if (row<0) row = 0;
				//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
				// set pixel color
				//*(char *)pixel_ptr = pixel_color;
				VGA_PIXEL(col,row,pixel_color);	
			}
					
		}
}

/****************************************************************************************
 * Draw a  circle on the VGA monitor 
****************************************************************************************/

void VGA_circle(int x, int y, int r, int pixel_color)
{
	char  *pixel_ptr ; 
	int row, col, rsqr, xc, yc;
	int col1, row1;
	rsqr = r*r;
	
	for (yc = -r; yc <= r; yc++){
		//row = yc;
		col1 = (int)sqrt((float)(rsqr + r - yc*yc));
		// right edge
		col = col1 + x; // add the center point
		row = yc + y; // add the center point
		//check for valid 640x480
		if (col>639) col = 639;
		if (row>479) row = 479;
		if (col<0) col = 0;
		if (row<0) row = 0;
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);	
		// left edge
		col = -col1 + x; // add the center point
		//check for valid 640x480
		if (col>639) col = 639;
		if (row>479) row = 479;
		if (col<0) col = 0;
		if (row<0) row = 0;
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);	
	}
	for (xc = -r; xc <= r; xc++){
		//row = yc;
		row1 = (int)sqrt((float)(rsqr + r - xc*xc));
		// right edge
		col = xc + x; // add the center point
		row = row1 + y; // add the center point
		//check for valid 640x480
		if (col>639) col = 639;
		if (row>479) row = 479;
		if (col<0) col = 0;
		if (row<0) row = 0;
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);	
		// left edge
		row = -row1 + y; // add the center point
		//check for valid 640x480
		if (col>639) col = 639;
		if (row>479) row = 479;
		if (col<0) col = 0;
		if (row<0) row = 0;
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);	
	}
}

// =============================================
// === Draw a line
// =============================================
//plot a line 
//at x1,y1 to x2,y2 with color 
//Code is from David Rodgers,
//"Procedural Elements of Computer Graphics",1985
void VGA_line(int x1, int y1, int x2, int y2, short c) {
	int e;
	signed int dx,dy,j, temp;
	signed int s1,s2, xchange;
     signed int x,y;
	char *pixel_ptr ;
	
	/* check and fix line coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
        
	x = x1;
	y = y1;
	
	//take absolute value
	if (x2 < x1) {
		dx = x1 - x2;
		s1 = -1;
	}

	else if (x2 == x1) {
		dx = 0;
		s1 = 0;
	}

	else {
		dx = x2 - x1;
		s1 = 1;
	}

	if (y2 < y1) {
		dy = y1 - y2;
		s2 = -1;
	}

	else if (y2 == y1) {
		dy = 0;
		s2 = 0;
	}

	else {
		dy = y2 - y1;
		s2 = 1;
	}

	xchange = 0;   

	if (dy>dx) {
		temp = dx;
		dx = dy;
		dy = temp;
		xchange = 1;
	} 

	e = ((int)dy<<1) - dx;  
	 
	for (j=0; j<=dx; j++) {
		//video_pt(x,y,c); //640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (y<<10)+ x; 
		// set pixel color
		//*(char *)pixel_ptr = c;
		VGA_PIXEL(x,y,c);			
		 
		if (e>=0) {
			if (xchange==1) x = x + s1;
			else y = y + s2;
			e = e - ((int)dx<<1);
		}

		if (xchange==1) y = y + s2;
		else x = x + s1;

		e = e + ((int)dy<<1);
	}
}

void VGA_Sine(short c) {
    float y;
    int x;
    int x1 = 0;
    int x2 = 1000;
    int amplitude = 3;
    
    // Ensure x coordinates are within screen bounds
    if (x1 < 0) x1 = 0;
    if (x2 > 639) x2 = 639;

    // Loop over x coordinates from x1 to x2
    for (x = x1; x <= x2; x++) {
        // Calculate y as a sine function of x
        y = amplitude * sin(0.1 * x); // The 0.1 is a scaling factor for the x value

        // Adjust y to be in the middle of the screen (240 is the center for 480 height)
        y += 240;

        // Ensure y coordinate is within screen bounds
        if (y < 0) y = 0;
        if (y > 479) y = 479;

        // Plot the point at (x, y) with color c
        VGA_PIXEL(x, (int)y, c);
    }
}

///////////////////////////////////////////////////////////////
// read the keyboard
///////////////////////////////////////////////////////////////
void * read1()
{
		while(1){
                //wait for print done
				pthread_mutex_lock(&print_lock);
				pthread_cond_wait(&print_cond,&print_lock);
				// the actual enter				
                printf("Enter a string: ");
                scanf("%s",input_buffer);
				// unlock the input_buffer
                pthread_mutex_unlock(&print_lock);
			    // and tell write1 thread that enter is complete
                pthread_cond_signal(&enter_cond);
        } // while(1)
}

///////////////////////////////////////////////////////////////
// write the input string
///////////////////////////////////////////////////////////////
void * write1() {
		sleep(1);
		// signal that the print process is ready when started
		pthread_cond_signal(&print_cond);
		
        while(1){
				// wait for enter done
                pthread_mutex_lock(&enter_lock);
                pthread_cond_wait(&enter_cond,&enter_lock);
				// the protected print (with protected counter)
				pthread_mutex_lock(&count_lock);
                printf("The string entered is %s, %d\n",input_buffer, count1);
				count1 = 0;
				pthread_mutex_unlock(&count_lock);
				// unlock the input_buffer
				pthread_mutex_unlock(&enter_lock);
                // and tell read1 thread that print is done
				pthread_cond_signal(&print_cond);         
        } // while(1)
}

///////////////////////////////////////////////////////////////
// counter 1
///////////////////////////////////////////////////////////////
// with mutex, about 9 to 10 million/sec
// -- adding a second copy drops rate to about 4 million/sec
// -- adding a second copy pegs BOTH processors at 100%
// without mutex about 200 to 400 millon/sec
// 
void * counter1() {
		//
        while(1){
				// count as fast as possible
				pthread_mutex_lock(&count_lock);
                count1++;    
				pthread_mutex_unlock(&count_lock);
				              
        } // while(1)
}

///////////////////////////////////////////////////////////////
