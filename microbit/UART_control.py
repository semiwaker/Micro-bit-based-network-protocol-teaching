from microbit import i2c, sleep, reset, Image, display,\
                     uart, pin12, pin13, button_a, compass, accelerometer
import struct
import ustruct

ID = ord(b'A')
ID_text = 'A'


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
speed = [0] * 4
uart.init(baudrate=115200, tx=pin12, rx=pin13)
display.show([ID_text])

while not button_a.is_pressed():
    display.show([ID_text])
display.show(Image.SMILE)
sleep(500)

# music.play(music.RINGTONE)

uart.write("Init   %c" % ID_text)

buf = b''

while True:
    display.show([ID_text])
    if uart.any():
        s = uart.read(8)
        buf = buf + s
        if len(buf) < 8:
            continue
        s = buf
        buf = b''
        display.show(Image.HEART)

        data = struct.unpack('8s', s)

        if data[0][0] != ID:
            continue

        if data[0][1] == ord(b'S'):
            for i in range(4, 8, 1):
                speed[i-4] = data[0][i]
            set_car_speed()

        st = struct.pack('4s4B3i3i',
                         data[0][0:4],
                         speed[0],
                         speed[1],
                         speed[2],
                         speed[3],
                         compass.get_x(),
                         compass.get_y(),
                         compass.get_z(),
                         accelerometer.get_x(),
                         accelerometer.get_y(),
                         accelerometer.get_z()
                         )
        uart.write(st)

        if data[0][1] == ord(b'R'):
            display.show(Image.SAD)
            for i in range(4, 8, 1):
                speed[i-4] = 0
            set_car_speed()
            sleep(1000)
            reset()
