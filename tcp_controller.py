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
import socket
import struct
import random
import time
import math
import numpy as np


class car_controller:
    """
    A class to hold tcp connection,
    receive key press to control the robot.
    """

    def __init__(self, server, client, viewer):
        '''
        This class is normaly created by car_server
        '''

        self.server = server
        self.client = client
        self.viewer = viewer

        self.left_rate = 1.0
        self.right_rate = 1.0
        # The left_rate and right_rate param is for adjusting the difference between the two wheels.

        self.fixed = 0

        self.get_init_msg()
        client.settimeout(3)

        self.straight = 0  # staight = 1/0/-1 means go forward/stand still/go backward
        self.turn = 0  # turn = 1/0/-1 means turn right/go staight/turn left

        self.query = False  # query = True if we need to send a query command
        self.restart = False  # restart = True if we need to send a restart command
        self.fix = False  # fix = True if we need to fix the data of the compass and accelerometer

        self.ID = status("ID", self.car_ID)
        self.PWMS = status("PWMS", [0, 0, 0, 0])
        self.accelerometers = status("Accelerometers", [0, 0, 0])
        self.compass = status("Compass", [0, 0, 0])
        self.acceleration = status("Acceleration", [0, 0])
        self.angle = status("Angle", 0.0)

        self.viewer.add_status(self.ID)
        self.viewer.add_status(self.PWMS)
        self.viewer.add_status(self.compass)
        self.viewer.add_status(self.accelerometers)
        self.viewer.add_status(self.acceleration)
        self.viewer.add_status(self.angle)

        self.viewer.set_onKey(self.onKey)

        try:
            self.communicate(order=b'Q')
        except OSError:
            viewer.print("Connection error.")
            return

        self.c_start = np.array([self.compass.get()[0], self.compass.get()[2]])  # The start state of the compass
        self.a_start = np.array([self.accelerometers.get()[0], self.accelerometers.get()[2]])
        # The start state of the accelerometer

        self.c_bias = None  # The calculated bias of the compass
        self.a_bias = None  # The calculated bias of the acclerometer
        self.a_ground = None
        # A component of the accleration, which is cause by the tilt of the ground

    def __del__(self):
        self.client.close()

    def onKey(self, key):
        " This method is a callback when a key is pressed "
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
        elif key == ord('r'):
            self.viewer.print("Restart")
            self.restart = True
        elif key == ord('f'):
            self.viewer.print("Fix")
            self.fix = True

        try:
            self.update()
        except OSError:
            self.viewer.print("Disconnected")
            self.reconnect()

    def update(self):
        "Send the request command by the last key press, refresh the status"
        # TODO: modify this method for your needs
        if self.restart:
            # Send a restart command
            self.communicate(order=b"R")
            self.restart = False
        elif self.query:
            # Send a query command
            self.communicate(order=b'Q')
            self.query = False
        elif self.fix:
            # Send a query command and calculate the bias
            self.communicate(order=b'Q')
            self.fix_bias()
            self.fix = False
        else:
            # send a set command

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
                        time.sleep(0.5)
                # Failed too much times, there should be a connection error
                raise OSError
            return wrapper

    def calc_precise_data(self):
        if not self.fixed:
            return

        c = self.compass.get()
        c = np.array([c[0], c[2]])
        a = self.accelerometers.get()
        a = np.array([a[0], a[2]])

        c_fixed = c - self.c_bias

        angle = math.acos(np.dot(c_fixed, self.c_start) / np.linalg.norm(c_fixed) / np.linalg.norm(self.c_start))
        if np.cross(c_fixed, self.c_start) < 0.0:
            angle *= -1.0

        self.angle.set_value(angle)

        def rotate(v, theta):
            return np.array(v[0] * math.cos(theta) - v[1] * math.sin(theta),
                            v[0] * math.sin(theta) + v[1] * math.cos(theta))

        a_fixed = a - self.a_bias - rotate(self.a_ground, -angle)
        self.acceleration.set_value(a_fixed)

    def show_received(self, msg, received):
        ' A helper method to update the statuses'
        self.viewer.print("Sent: "+str(msg))
        self.viewer.print("Received: "+str(received))

        self.PWMS.set_value(received[4:8])
        self.compass.set_value(received[8:11])
        self.accelerometers.set_value(received[11:14])

    @retry_when_failed(10)
    def communicate(self, order, PWMS=[0, 0, 0, 0]):
        ' Send a command to the robot and expects a correct response'

        # Some random padding bytes, expects the same in reponse
        space3 = bytes(chr(random.choice(range(32, 128))), "utf-8")
        space4 = bytes(chr(random.choice(range(32, 128))), "utf-8")

        # Pack the command into bytes
        msg = struct.pack("4c4B", self.car_ID, order, space3, space4, *PWMS)

        # Sending the command
        try:
            self.send(msg)
            time.sleep(0.05)
            received = struct.unpack("4c4B6i", self.read(32))
        except socket.timeout:
            self.viewer.print("Timeout")
            return False

        # Validate the response
        if (received[0] != self.car_ID or received[1] != order
           or received[2] != space3 or received[3] != space4):
            self.viewer.print("Wrong reply:")
            self.viewer.print(str(received))
            return False

        self.show_received(msg, received)
        self.calc_precise_data()
        return True

    @retry_when_failed(10000)
    def get_init_msg(self):
        'Read the init message from the robot and get its ID'
        # Send nothing, reply format: "Init   {ID}"
        try:
            init_msg = self.read(8)
        except socket.timeout:
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

    def fix_bias(self):
        'Calculate the bias to get the precise data of the compass and acclerometer'
        c = self.compass.get()
        c = np.array([c[0], c[2]])
        a = self.accelerometers.get()
        a = np.array([a[0], a[2]])

        self.c_bias = (self.c_start + c) / 2.0
        self.c_start = self.c_start - self.c_bias

        self.a_bias = (self.a_start + a) / 2.0
        self.a_ground = self.a_start - self.a_bias

        self.fixed = 1

    def send(self, msg):
        'Send the message in its entirety.'
        total_sent = 0
        while total_sent < len(msg):
            sent = self.client.send(msg[total_sent:])
            total_sent += sent

    def read(self, byte_cnt):
        'Read byte_cnt of bytes.'
        chunks = []
        while byte_cnt > 0:
            chunk = self.client.recv(byte_cnt)
            chunks.append(chunk)
            byte_cnt -= len(chunk)
        return b''.join(chunks)

    def reconnect(self):
        'Try to reconnect.'
        try:
            self.client, _ = self.server.accept()
        except OSError:
            viewer.print("Reconnection failed")
            viewer.wait_key()
            viewer.stop()
            return
        viewer.print("Reconnected")

    def set_left_rate(self, rate):
        self.left_rate = rate

    def set_right_rate(self, rate):
        self.right_rate = rate


class car_server:
    """
    A class to hold tcp server
    Accept connection and return car_controller
    """

    def __init__(self, viewer, IP=None, port=18302):
        self.viewer = viewer
        try:
            self.server = socket.socket()

            if IP is None:
                IP = socket.gethostbyname(socket.gethostname())

            self.server.bind((IP, port))
            self.server.listen()
        except OSError:
            self.viewer.print(f"Server failed.")
            self.viewer.wait_key()
            return

        self.viewer.print(f"Server created. IP:{IP}, port:{port}")

    def __del__(self):
        self.server.close()

    def connect_car(self):
        self.viewer.print("Waiting for connection.")

        client, client_address = self.server.accept()

        self.viewer.print(f"Connection accepted from {client_address}")

        return car_controller(server=self.server, client=client, viewer=self.viewer)


if __name__ == "__main__":
    viewer = status_viewer(title="TCP controller")
    server = car_server(viewer=viewer, IP="192.168.137.1")
    controller = server.connect_car()
    viewer.begin()
