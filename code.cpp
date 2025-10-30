#include <iostream>
#include <chrono>
#include <unistd.h> // For usleep/sleep
#include <pigpio.h>
#include <cstdio>
#include <unistd.h>
#include <sys/types.h>
#include <arpa/inet.h> // Required for ntohl (Network To Host Long)
#include <cstdint>     // Required for uint32_t
// --- Hardware Definitions ---
// Use WiringPi BCM mode numbers
#define DC_PIN 6
#define RST_PIN 12

// SPI Configuration
#define SPI_CHANNEL 0
#define SPI_SPEED_HZ 2200000 
#define SPI_MODE 0
#define SPI_FLAGS 0
#define WIDTH 400
#define HEIGHT 600
#define bufsize 30300
const size_t RAW_COLOR_SIZE = 600 * 400 * 3;
void display( char *data, int len) ;
char framebuffer[bufsize];
// --- Global SPI variable ---
// WiringPiSPI handles the SPI device file descriptor internally.
int spi_handle;
// Function to send a command byte array
void command( char *data, int len) {
    gpioWrite(DC_PIN, 0); // Set DC pin LOW for command
    // wiringPiSPIDataRW handles both reading and writing the data
    spiWrite(spi_handle, data, len);
}
void render(){
    size_t current_offset=0;
    while(current_offset<bufsize){
	    size_t chunk_len=std::min((size_t)4096,bufsize-current_offset);
	    display(framebuffer+current_offset,chunk_len);
	    current_offset+=chunk_len;
    }
}
const uint8_t PIXEL_LOOKUP_TABLE[4][2] = {
    {1, 0}, // y%4 = 0
    {3, 2}, // y%4 = 1
    {5, 4}, // y%4 = 2
    {7, 6}  // y%4 = 3
};
void set_pixel(int x, int y, bool set_on){
	int cellx=x/2;
	int celly=y/12;
	int offset_y=y%4;
	int offset_x=x%2;
	int ybase=celly*606;
	int xbase=cellx*3;
	int index=ybase+xbase;
	int byteoffset=2-((y%12)/4);
	int bit_pos=PIXEL_LOOKUP_TABLE[offset_y][offset_x];
	int final_byte_index=index+byteoffset;
	char bitmask=(1<<bit_pos);
	if (set_on){
		framebuffer[final_byte_index]|=bitmask;
	}else{
		framebuffer[final_byte_index]&=~bitmask;
	}
}
// Function to send display data byte array
void display( char *data, int len) {
    gpioWrite(DC_PIN, 1); // Set DC pin LOW for command
    // wiringPiSPIDataRW handles both reading and writing the data
    spiWrite(spi_handle, data, len);
}

char buf[4096]={};
struct headerstart{
	uint32_t header_size;
	  uint32_t  FileVersion;        /* X10 XWD file version (always 06h) */
	  uint32_t DisplayType;        /* Display type */
	  uint32_t DisplayPlanes;      /* Number of display planes */
	  uint32_t PixmapFormat;       /* Pixmap format */
	  uint32_t PixmapWidth;        /* Pixmap width */
	  uint32_t PixmapHeight;       /* Pixmap height */
	  uint16_t WindowWidth;        /* Window width */
	  uint16_t WindowHeight;       /* Window height */
	  uint16_t WindowX;            /* Window upper left X coordinate */
	  uint16_t WindowY;            /* Window upper left Y coordinate */
	  uint16_t WindowBorderWidth;  /* Window border width */
	  uint16_t WindowNumColors;    /* Number of color entries in window */
};
void capture_and_mirror() {
    // Command to execute: xwd captures the root window of display :1 and pipes the output
    const char* xwd_command = "xwd -root -display :0";
    FILE* pipe = popen(xwd_command, "r");
    if (!pipe) {
        std::cerr << "ERROR: Could not run xwd command." << std::endl;
        return;
    }
    headerstart header_info;
    if (fread(&header_info, sizeof(headerstart), 1, pipe)!=1){
	    std::cerr<<"error frailed to read header";
	    return ;
    }
    uint32_t raw_header_size=ntohl(header_info.header_size);
    uint32_t numcolors=ntohl(header_info.WindowNumColors);
   

    // 1. Skip the XWD Header (Header size is highly dependent on X server/setup, typically ~100 bytes)
    // You MUST determine the exact header size by running xwd once and checking the file size.
    // We assume a 100-byte header for this example, but it must be precise.
	//fseek(pipe, raw_header_size, SEEK_CUR);
	//3024
	
	char skip_buf[raw_header_size];
	fread(skip_buf, 1, raw_header_size, pipe);


    // 2. Read the raw RGB pixel data (600*400*3 bytes)
    uint8_t raw_color_pixels[RAW_COLOR_SIZE];
    if (fread(raw_color_pixels, 1, RAW_COLOR_SIZE, pipe) != RAW_COLOR_SIZE) {
        std::cerr << "ERROR: Failed to read expected amount of pixel data." << std::endl;
        pclose(pipe);
        return;
    }
    pclose(pipe); 

    // 3. Process Color Data and Pack into ST7306 Framebuffer
    // Clear the ST7306 frame_buffer first!
    for (int i=0;i<bufsize;i++){framebuffer[i]=0;}

    for (int y = 0; y < HEIGHT; ++y) {
	
        for (int x = 0; x < WIDTH; ++x) {
            // Index into the flat raw_color_pixels array (3 bytes per pixel: R, G, B)
            size_t idx = (y * WIDTH + x) * 3;
            
            uint8_t r = raw_color_pixels[idx];
            uint8_t g = raw_color_pixels[idx + 1];
            uint8_t b = raw_color_pixels[idx + 2];
            
            // Luminance/Monochrome Conversion: Simple average threshold
            // If the color is bright enough (over a threshold), set the pixel ON (True).
            // A more accurate luminance conversion: 0.2126*R + 0.7152*G + 0.0722*B
            if (r + g + b > 100) { // 384 = (255 * 3) / 2 (mid-gray threshold)
                set_pixel(x, y, true);
                set_pixel(x, y+1, true);
            }
        }
    }
}
int main() {
    // 1. Initialize WiringPi and set pin modes (using BCM)
    // IMPORTANT: wiringPiSetupGpio() uses BCM numbering (like RPi.GPIO.BCM)
	if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio." << std::endl;
        return 1;
    }

    gpioSetMode(DC_PIN, PI_OUTPUT);
    gpioSetMode(RST_PIN, PI_OUTPUT);

    // 2. Initialize SPI
    // Sets speed, mode, and opens the device file /dev/spidev0.0
    spi_handle = spiOpen(SPI_CHANNEL, SPI_SPEED_HZ, SPI_FLAGS);
    // 3. Hardware Reset (Reset sequence translated to C++)
    gpioWrite(RST_PIN, 0);
    // Since DC is controlled by command/display functions, we set it LOW here too for the reset sequence's initial state.
    gpioWrite(DC_PIN, 0); 
    time_sleep(0.300);

    gpioWrite(RST_PIN, 1);
    time_sleep(0.300);

    // 4. Power and Timing Commands (Original sequence)
    // std::vector<uint8_t> is used to pass array data
    buf[0]=0xD1;
    command(buf,1);
    buf[0]=0x01;
    display(buf,1);

    buf[0]=0xB8;
    command(buf,1);
    buf[0]=0x45;
    display(buf,1);

    buf[0]=0x11;
    command(buf,1);

    time_sleep(0.300);

    buf[0]=0xD8;
    command(buf,1);
    buf[0]=0xa6;
    buf[1]=0xE9;
    display(buf,2);


    buf[0]=0xB2;
    command(buf,1);
    buf[0]=0x00;
    display(buf,1);

    buf[0]=0x36;
    command(buf,1);
    buf[0]=0b0101000;
    display(buf,1);

    buf[0]=0x3A;
    command(buf,1);
    buf[0]=0x11;
    display(buf,1);

    buf[0]=0x2B;
    command(buf,1);
    buf[0]=0x00;
    buf[1]=0xC9;
    display(buf,2);

    buf[0]=0x2A;
    command(buf,1);
    buf[0]=0x05;
    buf[1]=0x36;
    display(buf,2);

    buf[0]=0x29;
    command(buf,1);

    buf[0]=0xBB;
    command(buf,1);
    buf[0]=0xcf;
    display(buf,1);
    time_sleep(0.110);

    buf[0]=0x2C;
    command(buf,1);

    while (true){
	    auto start_capture = std::chrono::high_resolution_clock::now();
capture_and_mirror();
    render();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_capture).count();
    usleep(50000);

    }

    time_sleep(10);
    gpioWrite(RST_PIN, 0);
    spiClose(spi_handle);
    gpioTerminate(); // Disconnect from pigpio daemon
    return 0;
}
