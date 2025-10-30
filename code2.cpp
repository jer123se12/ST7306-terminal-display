#include <iostream>
#include <chrono>
#include <unistd.h>
#include <pigpio.h>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <cstdlib> // For system()
#include <arpa/inet.h> // Required for ntohl (Network To Host Long)
#include <cstdint>     // Required for uint32_t
#include "lcddriver.c"
#include "font8x8/font8x8.h"
#define FONTSIZE 1
int cursor[]={0,0};

#define XMULT (FONTSIZE * 8)
#define YMULT (FONTSIZE * 16)

// Note: Integer division is performed here, just like with 'static int'
#define TCOLS (WIDTH / XMULT)
#define TROWS (HEIGHT / YMULT)

#define BX ((WIDTH - (TCOLS * XMULT)) / 2)
#define BY ((HEIGHT - (TROWS * YMULT)) / 2)

#define BUFFER_SIZE (4 + (TROWS * TCOLS * 2))
char console_data[BUFFER_SIZE+4];
void setchar(int startx,int starty,char *bitmap);
void setchar(int* cursor,char *bitmap);
void setchar(int cursor[2],char *bitmap){
	if (cursor[0]>=TCOLS){
		cursor[1]++;
		cursor[0]%=TCOLS;
	}
//	cursor[1]%=TROWS;


	
	setchar(BX+(cursor[0] * XMULT),BY+(cursor[1] * YMULT),bitmap);


}
void setchar(int startx,int starty,char *bitmap){
	int x,y;
	int set;
	int mask;
	for (x=0;x<8;x++){
		for (y=0;y<8;y++){
			int basex=startx+(x*FONTSIZE);
			int basey=starty+(y*FONTSIZE*2);
			bool value=bitmap[y]&1<<x;
			for (int i=0;i<FONTSIZE;i++){
				for (int j=0;j<FONTSIZE*2;j++){
					set_pixel(basex+i,basey+j,value);
				}
			}
		}
	}
}
void setmsg(int startx,int starty,char * message,int length){
	for (int i=0;i<length;i++){
		int xpos=(i*XMULT);
		setchar(startx+xpos,starty,font8x8_basic[message[i]]);

	}
		

}
int vcsa_fd;
void init_terminal(){
	char command[64];
	// Set rows
	sprintf(command, "stty --file=/dev/console rows %d", TROWS);
	system(command);
	// Set columns
	sprintf(command, "stty --file=/dev/console cols %d", TCOLS);
	system(command);
	 vcsa_fd= open("/dev/vcsa1", O_RDONLY);
}
void read_console_buffer() {
    if (vcsa_fd >= 0) {
        // Seek to the beginning and read the whole block
        lseek(vcsa_fd, 0, SEEK_SET);
        read(vcsa_fd, console_data, BUFFER_SIZE);

        // The display data starts at index 4
	int rows = (int)console_data[0];
	int cols = (int)console_data[1];
	int length=rows*cols*4;
	cursor[0]=0;
	cursor[1]=0;
        char* screen_buffer = console_data+4;
	for (int i=0;i<BUFFER_SIZE;i+=2){
		char ch=screen_buffer[i];
		switch(ch){
			case '\r':
				cursor[0]=0;
				break;
			case '\n':
				cursor[1]++;
				cursor[0]=0;
				break;
			case '\b':
				cursor[0]--;
				setchar(cursor,font8x8_basic[32]);
				break;
			case '\t':
				setchar(cursor,font8x8_basic[32]);
				cursor[0]++;
				setchar(cursor,font8x8_basic[32]);
				cursor[0]++;
				break;
			default:
				setchar(cursor,font8x8_basic[ch]);
				cursor[0]++;


		}
		//render();
		//time_sleep(.5);

	}
//	for (int y=0;y<TROWS;y++){
//		for (int x=0;x<TCOLS;x++){
//			cursor[0]=x;
//			cursor[1]=y;
//			char ch=screen_buffer[(y*TROWS+x)*2];
//			setchar(cursor,font8x8_basic[ch]);
//		}
//	}

        // Example: Character at (r, c) is at: screen_buffer[ (r * COLS + c) * 2 ]
        // Attribute at (r, c) is at: screen_buffer[ (r * COLS + c) * 2 + 1 ]

        // ... (Proceed to Phase 3: Drawing) ...
    }
}
char * message="HELLO";
int main() {
    init_lcd();
    init_terminal();
    time_sleep(1);
    int i=0;
    //rendermsg(0,0,message,5);
    auto cur = std::chrono::high_resolution_clock::now();
    auto last=cur;
    while (true){
	    cur = std::chrono::high_resolution_clock::now();
	    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(cur - last).count();
	    i++;
	    //cursor[0]++;
	    //setchar(cursor,font8x8_basic[i%127]);
	    if (duration_ms>100){
		    last=std::chrono::high_resolution_clock::now();
		    read_console_buffer();
		    i=0;
		    render();
	    }
    }


    time_sleep(10);
    gpioWrite(RST_PIN, 0);
    spiClose(spi_handle);
    gpioTerminate(); // Disconnect from pigpio daemon
    return 0;
}
