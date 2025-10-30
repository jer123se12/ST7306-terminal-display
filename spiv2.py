
import time
import spidev
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
dc=6
rst=12
GPIO.setup(dc,GPIO.OUT)
GPIO.setup(rst,GPIO.OUT)

def command(msg):
    GPIO.output(dc,GPIO.LOW)
    spi.xfer2(msg)

def display(msg):
    GPIO.output(dc,GPIO.HIGH)
    spi.xfer2(msg)
# We only have SPI bus 0 available to us on the Pi
bus = 0

#Device is the chip select pin. Set to 0 or 1, depending on the connections
device = 0

# Enable SPI
spi = spidev.SpiDev()

# Open a connection to a specific bus and device (chip select pin)
spi.open(bus, device)

# Set SPI speed and mode
spi.max_speed_hz = 2200000
spi.mode = 0

# hw reset
GPIO.output(rst,GPIO.LOW)
GPIO.output(dc,GPIO.LOW)
time.sleep(0.3)

GPIO.output(rst,GPIO.HIGH)
time.sleep(0.3)

# power and timing
command([0xD1])
display([0x01])

command([0xB8])
display([0x45])
command([0x11])
time.sleep(0.12)
command([0xD8])
display([0xa6, 0xE9])

command([0xB2])
display([0x00])
command([0x36])
display([0b0101000])
command([0x3A])
display([0x11])

command([0x2B])
display([0x00])
#display([0x64])
display([0xc9])

command([0x2A])
display([0x05])
display([0x36])
#display([0x36])

command([0x29])
command([0xBB])
display([0xcf])
time.sleep(0.12)
command([0x2C])

WIDTH=400
HEIGHT=600
screen=[[False for i in range(WIDTH)] for i in range(HEIGHT)]


for i in range(len(screen)):
    for j in range(len(screen[0])):
        screen[j][j]=True

for i in range(min(HEIGHT,WIDTH)):
    screen[i][WIDTH-1-i]=True



start =time.time()
lookup=[
        [1,0],
        [3,2],
        [5,4],
        [7,6]
        ]
data=[False for i in range(202*50*3)]
for x in range(0,WIDTH):
    for y in range(0,HEIGHT):
        if screen[y][x]:
            index=(x//2)*3+((y//12)*(202*3))
            offsetx=x%2
            offsety=y%4
            byte=2-((y%12)//4)
            bitshift=1<<lookup[offsety][offsetx]
            data[index+byte]=data[index+byte]|bitshift
j=0
for i in range(1, len(data)):
    if i%4096==0:
        display(data[j:i])
        j=i
display(data[j:])
end=time.time()
print(end-start)
time.sleep(10)




GPIO.cleanup




        



