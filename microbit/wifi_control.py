from microbit import i2c, sleep, reset, Image, display,\
                     uart, pin12, pin13, pin16, button_a, compass,\
                     accelerometer
import music
import struct
import ustruct
import neopixel
np = neopixel.NeoPixel(pin16, 3)
np.clear()
del np
del neopixel

ID = ord(b'A')
ID_text = 'A'

IP = '"192.168.137.1"'
PORT = "18302"

display.show([ID_text])
while not button_a.is_pressed():
    pass


class PCA9685(object):
    def __init__(self, i2c, address=0x41):
        self.address = address
        i2c.write(self.address, bytearray([0x00, 0x00]))
        self.set_all_pwm(0, 0)
        i2c.write(self.address, bytearray([0x01, 0x04]))
        i2c.write(self.address, bytearray([0x00, 0x01]))
        sleep(5)
        i2c.write(self.address, bytearray([0x00]))
        mode1 = i2c.read(self.address, 1)
        mode1 = ustruct.unpack('<H', mode1)[0]
        mode1 = mode1 & ~0x10
        i2c.write(self.address, bytearray([0x00, mode1]))
        sleep(5)

    def set_pwm(self, channel, on, off):
        i2c.write(self.address, bytearray([0x06+4*channel, on & 0xFF]))
        i2c.write(self.address, bytearray([0x07+4*channel, on >> 8]))
        i2c.write(self.address, bytearray([0x08+4*channel, off & 0xFF]))
        i2c.write(self.address, bytearray([0x09+4*channel, off >> 8]))

    def set_all_pwm(self, on, off):
        i2c.write(self.address, bytearray([0xFA, on & 0xFF]))
        i2c.write(self.address, bytearray([0xFB, on >> 8]))
        i2c.write(self.address, bytearray([0xFC, off & 0xFF]))
        i2c.write(self.address, bytearray([0xFD, off >> 8]))


def set_car_speed(): 
    pwm.set_pwm(12, 0, speed[0] * 16)
    pwm.set_pwm(13, 0, speed[1] * 16)
    pwm.set_pwm(14, 0, speed[2] * 16)
    pwm.set_pwm(15, 0, speed[3] * 16)

pwm = PCA9685(i2c)
for i in range(1, 6, 1):
    pwm.set_pwm(i, 0, 0)
speed = [0] * 4
uart.init(baudrate=115200, tx=pin12, rx=pin13)

def send_AT(mSend, mRec1, mRec2=b""):
    while True:
        uart.write(mSend)
        sleep(100)
        if uart.any():
            s = uart.readall()
            if s == mRec1 or s == mRec2:
                break
    display.show(Image.HAPPY)
    sleep(200)

display.show(['1'])
send_AT(b"AT\r\n", b"AT\r\r\n\r\nOK\r\n")
display.show(['2'])
send_AT(b"AT+CIPMODE=1\r\n", b"AT+CIPMODE=1\r\r\n\r\nOK\r\n")
display.show(['3'])
send_AT(b'AT+CIPSTART="TCP",'+IP+','+PORT+'\r\n',
        b'AT+CIPSTART="TCP",'+IP+','+PORT+'\r\r\nCONNECT\r\n\r\nOK\r\n',
        b'AT+CIPSTART="TCP",'+IP+','+PORT+'\r\r\nALREADY CONNECT\r\n\r\nOK\r\n')
display.show(['4'])
send_AT(b"AT+CIPSEND\r\n", b"AT+CIPSEND\r\r\n\r\nOK\r\n\r\n>")

display.show([ID_text])

uart.write("Init   %c" % ID_text)

def deal_with_command(s):
    if s[0] != ID:
        return 0

    if s[1] != ord(b'S') and s[1]!=ord(b'R') and s[1]!=ord(b'Q'):
        return 0

    if s[1] == ord(b'S'):
        for i in range(4, 8, 1):
            speed[i-4] = s[i]
        set_car_speed()

    st = struct.pack('4s4B6i',
                     s[0:4],
                     speed[0],
                     speed[1],
                     speed[2],
                     speed[3],
                     compass.get_x(),
                     compass.get_y(),
                     compass.get_z(),
                     accelerometer.get_x(),
                     accelerometer.get_y(),
                     accelerometer.get_z())
    uart.write(st)
    
    sleep(20)

    if s[1] == ord(b'R'):
        display.show(Image.SAD)
        uart.write('+++')
        for i in range(4, 8, 1):
            speed[i-4] = 0
        set_car_speed()
        sleep(1000)
        reset()
    return 1

buf = b''

while True:
    if uart.any():
        s = uart.read(8)
        buf = (buf + s)[-8:]
        if len(buf) < 8:
            continue

        data = struct.unpack('8s', buf)
        if deal_with_command(data[0]): buf = b''

