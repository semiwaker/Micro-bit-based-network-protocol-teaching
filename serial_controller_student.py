# Author: Zizhang Luo
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


from status_viewer import status, status_viewer
import curses
import serial
import struct
import random
import time


class controller:
    """
    A class to hold serial connection,
    receive key press to control the robot.
    """

    def __init__(self, serial_name, viewer, left_rate=1.0, right_rate=1.0):
        '''
        Please look up the name of the serial device and fill into the serial_name param.
        The left_rate and right_rate param is for
        adjusting the difference between the two wheels.
        '''
        try:
            self.ser = serial.Serial(serial_name, baudrate=115200, timeout=0.5)
        except IOError:
            viewer.print("Wrong serial name! Please restart!")
            return

        self.left_rate = left_rate
        self.right_rate = right_rate
        self.viewer = viewer

        self.get_init_msg()

        self.straight = 0  # staight = 1/0/-1 means go forward/stand still/go backward
        self.turn = 0  # turn = 1/0/-1 means turn right/go staight/turn left

        self.query = False  # query = True if we need to send a query command

        self.ID = status("ID", self.car_ID)
        self.PWMS = status("PWMS", [0, 0, 0, 0])

        self.viewer.add_status(self.ID)
        self.viewer.add_status(self.PWMS)

        self.viewer.set_onKey(self.onKey)

    def __del__(self):
        self.ser.close()

    def onKey(self, key):
        " This method is a callback when the key is pressed "
        # TODO: Modify this method to suit your needs

        if key == curses.KEY_UP:
            self.viewer.print("Up")
            self.straight += 1 if self.straight < 1 else 0
        elif key == curses.KEY_DOWN:
            self.viewer.print("Down")
            self.straight -= 1 if self.straight > -1 else 0
        elif key == curses.KEY_LEFT:
            self.viewer.print("Left")
            self.turn -= 1 if self.turn > -1 else 0
        elif key == curses.KEY_RIGHT:
            self.viewer.print("Right")
            self.turn += 1 if self.turn < 1 else 0
        elif key == ord(' '):
            self.viewer.print("Stop")
            self.straight = 0
            self.turn = 0
        elif key == ord('q'):
            self.viewer.print("Query")
            self.query = True

        try:
            self.update()
        except serial.SerialException:
            self.viewer.print("Disconnected")
            self.viewer.wait_key()
            self.viewer.stop()

    def update(self):
        "Send the request command by the last key press, refresh the status"
        # TODO: modify this method for your needs
        if self.query:
            # Send a query command
            self.communicate(order=b'Q')
            self.query = False
        else:
            # Send a set command

            # Caculate the left and right PWM
            turn_rate_l = {-1: 0.9, 0: 1.0, 1: 1.3}
            turn_rate_r = {-1: 1.3, 0: 1.0, 1: 0.9}
            lpwm = 120 * turn_rate_l[self.turn] * self.left_rate
            rpwm = 120 * turn_rate_r[self.turn] * self.right_rate
            PWMS = [0, 0, 0, 0]
            if self.straight == 0:
                modify = {-1: (-1, 1), 0: (0, 0), 1: (1, -1)}
                lpwm *= modify[self.turn][0]
                rpwm *= modify[self.turn][1]
            if self.straight < 0:
                lpwm *= -1
                rpwm *= -1
            # Transform the lpwm and rpwm into the 4 pwms to control the robot
            PWMS[0], PWMS[1] = (lpwm, 0) if lpwm >= 0 else (0, -lpwm)
            PWMS[2], PWMS[3] = (0, rpwm) if rpwm >= 0 else (-rpwm, 0)
            # Make sure that the pwms are int
            PWMS = list(map(int, PWMS))
            self.communicate(order=b'S', PWMS=PWMS)

    class retry_when_failed:
        'A decorator, retry if the wrapped function return false, for at most max_try times.'
        def __init__(self, max_try):
            self.max_try = max_try

        def __call__(self, func):
            def wrapper(*args, **kwargs):
                for t in range(self.max_try):
                    if func(*args, **kwargs):
                        return
                    else:
                        time.sleep(0.1)
                # Failed too much times, there should be a connection error
                raise serial.SerialException
            return wrapper

    def show_received(self, msg, received):
        ' A helper method to update the statuses'
        self.viewer.print("Sent: "+str(msg))
        self.viewer.print("Received: "+str(received))

        self.PWMS.set_value(received[4:8])

    @retry_when_failed(10)
    def communicate(self, order, PWMS=[0, 0, 0, 0]):
        ' Send a command to the robot and expects a correct response'

        # Some random padding bytes, expects the same in reponse
        space3 = bytes(chr(random.choice(range(0, 128))), "utf-8")
        space4 = bytes(chr(random.choice(range(0, 128))), "utf-8")

        # Pack the command into bytes
        msg = struct.pack("4c4B", self.car_ID, order, space3, space4, *PWMS)

        # Sending the command
        try:
            self.ser.flushInput()
            self.send(msg)
            time.sleep(0.01)
            received = struct.unpack("4c4B", self.read(8))
        except serial.SerialTimeoutException:
            self.viewer.print("Timeout")
            return False

        # TODO: Validate the response

        self.show_received(msg, received)
        return True

    @retry_when_failed(10000)
    def get_init_msg(self):
        'Read the init message from the robot and get its ID'
        # Send nothing, reply format: "Init   {ID}"
        try:
            init_msg = self.read(8)
        except serial.SerialTimeoutException:
            return False
        self.viewer.print("Init msg: "+str(init_msg))
        # Check the message
        if init_msg[:7] != b"Init   " or init_msg[7] < ord('A') or init_msg[7] > ord('Z'):
            self.viewer.print("Wrong init message!" + str(init_msg))
            time.sleep(3)
            self.ser.flushInput()
            return False
        else:
            self.car_ID = bytes(chr(init_msg[7]), "utf-8")
            return True

    def send(self, msg):
        'Send the message in its entirety'
        total_sent = 0
        while total_sent < len(msg):
            sent = self.ser.write(msg[total_sent:])
            if sent == 0:
                # Connection error
                raise serial.SerialException
            total_sent += sent
        self.ser.flush()

    def read(self, byte_cnt):
        'Read byte_cnt of bytes'
        chunks = []
        while byte_cnt > 0:
            chunk = self.ser.read(byte_cnt)
            chunks.append(chunk)
            byte_cnt -= len(chunk)
        return b''.join(chunks)


if __name__ == "__main__":
    print("Please input serial name:")
    name = input()
    viewer = status_viewer(title="UART controller")
    control = controller(serial_name=name, viewer=viewer)
    viewer.begin()
