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
spi.max_speed_hz = 3200000
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

#for i in range(99)
#    for i in range(300):
#        display([0x00])
#for i in range(18):
#    for i in range(30):
#        display([0x00])
#for i in range(55):
#    display([0x00])
# total refresh =10100*3
# row =202*3
arr=[]
#k=0xff
#i=0
#arr.append(0b11001100)
#arr.append(0b11001100)
#arr.append(0b11001100)
#display(arr)
def initframe():
    frame=[]
    for i in range(0,50):
        for j in range(202):
            frame.append(0x00)
            frame.append(0x00)
            frame.append(0x00)
    return frame
lookup=[
        [ 1, 0],
        [ 3, 2],
        [ 5, 4],
        [ 7, 6],
        [ 9, 8],
        [11,10],
        [13,12],
        [15,14],
        [17,16],
        [19,18],
        [21,20],
        [23,22],
        ]
def setpixel(x,y):
    index=(x//2)*3+((y//12)*(202*3))
    offx=x%2
    offy=y%12
    offset=lookup[offy][offx]
    bitshift=1<<(offset%8)
    if offset<8:
        frame[index+2]=frame[index+2]|bitshift
    elif offset<16:
        frame[index+1]=frame[index+1]|bitshift
    else:
        frame[index]=frame[index]|bitshift

frame=initframe()
for i in range(0,400):
    for j in range(0,400):
        setpixel(j,j)
j=0
for i in range(1,len(frame)):
    if i%4096==0:
        display(frame[j:i])
        j=i
        
display(frame[j::])
arr=[]
time.sleep(0.5)

#    for i in range(0,202*3):
#        j=0x0f if j==0xf0 else 0xf0
#        arr.append(j)
#        if i%4096==0:
#            display(arr)
#            arr=[]
#    display(arr)
#    arr=[]



GPIO.cleanup()
