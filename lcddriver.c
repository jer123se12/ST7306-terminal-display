
#include <iostream>
#include <chrono>
#include <unistd.h> // For usleep/sleep
#include <pigpio.h>
#include <cstdio>
#include <unistd.h>
#include <sys/types.h>
#include <arpa/inet.h> // Required for ntohl (Network To Host Long)
#include <cstdint>     // Required for uint32_t
#define DC_PIN 6
#define RST_PIN 12
#define OFFX 0x00
#define OFFY 0x05

// SPI Configuration
#define SPI_CHANNEL 0
#define SPI_SPEED_HZ 2200000 
#define SPI_MODE 0
#define SPI_FLAGS 0
#define WIDTH 400
#define HEIGHT 600
#define bufsize 30300
const size_t RAW_COLOR_SIZE = 600 * 400 * 3;
char framebuffer[bufsize];
char _framebuffer[bufsize];
int spi_handle;
char buf[4096]={};
void display( char *data, int len) {
    gpioWrite(DC_PIN, 1); 
    spiWrite(spi_handle, data, len);
}
void command( char *data, int len) {
    gpioWrite(DC_PIN, 0);
    spiWrite(spi_handle, data, len);
}
int getbaseindex(int x, int y){
	int cellx=x/2;
	int celly=y/12;
	int ybase=celly*606;
	int xbase=cellx*3;
	int index=ybase+xbase;
	return index;
}
void render(){
    size_t current_offset=0;
    while(current_offset<bufsize){
	    size_t chunk_len=std::min((size_t)4096,bufsize-current_offset);
	    display(framebuffer+current_offset,chunk_len);
	    current_offset+=chunk_len;
    }
    for (int i=0;i<bufsize;i++){
	_framebuffer[i]=framebuffer[i];
    }
}
const uint8_t PIXEL_LOOKUP_TABLE[4][2] = {
    {1, 0}, // y%4 = 0
    {3, 2}, // y%4 = 1
    {5, 4}, // y%4 = 2
    {7, 6}  // y%4 = 3
};
void set_pixel(int x, int y, bool set_on){
	
	int index=getbaseindex(x,y);
	int offset_y=y%4;
	int offset_x=x%2;
	int byteoffset=2-((y%12)/4);
	int bit_pos=PIXEL_LOOKUP_TABLE[offset_y][offset_x];
	int final_byte_index=index+byteoffset;
	if (final_byte_index>=bufsize){
		return;}
	char bitmask=(1<<bit_pos);
	if (set_on){
		framebuffer[final_byte_index]|=bitmask;
	}else{
		framebuffer[final_byte_index]&=~bitmask;
	}
}
void init_lcd(){

    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio." << std::endl;
    }

    gpioSetMode(DC_PIN, PI_OUTPUT);
    gpioSetMode(RST_PIN, PI_OUTPUT);

    spi_handle = spiOpen(SPI_CHANNEL, SPI_SPEED_HZ, SPI_FLAGS);
    gpioWrite(RST_PIN, 0);
    gpioWrite(DC_PIN, 0); 
    time_sleep(0.300);

    gpioWrite(RST_PIN, 1);
    time_sleep(0.300);

    buf[0]=0xD1;command(buf,1);
    buf[0]=0x01;display(buf,1);

    buf[0]=0xB8;command(buf,1);
    buf[0]=0x45;display(buf,1);

    buf[0]=0x11;command(buf,1);

    time_sleep(0.300);

    buf[0]=0xD8;command(buf,1);
    buf[0]=0xa6;buf[1]=0xE9;display(buf,2);


    buf[0]=0xB2;command(buf,1);
    buf[0]=0x00;display(buf,1);

    buf[0]=0x36;command(buf,1);
    buf[0]=0b0101000;display(buf,1);

    buf[0]=0x3A;command(buf,1);
    buf[0]=0x11;display(buf,1);

    buf[0]=0x2B;command(buf,1);
    buf[0]=0x00;buf[1]=0xC9;display(buf,2);

    buf[0]=0x2A;command(buf,1);
    buf[0]=0x05;buf[1]=0x36;display(buf,2);

    buf[0]=0x29;command(buf,1);

    buf[0]=0xBB;command(buf,1);
    buf[0]=0xcf;display(buf,1);
    time_sleep(0.110);

    buf[0]=0x2C;command(buf,1);
}

