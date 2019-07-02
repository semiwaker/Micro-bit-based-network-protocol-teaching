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


import curses
# In Windows, pip install windows-curses
import curses.ascii
from collections import deque


class status:
    "A class whose object represents a status to show"
    def __init__(self, name, obj):
        "Defien a status called name, with initial value obj"
        self.obj = obj
        self.name = name

    def get(self):
        "Get the value of the status"
        return self.obj

    def set_value(self, v):
        "Set the value of the status to v"
        self.obj = v


class status_viewer:
    """
    A class to show the specified status in the left side of the screen,
    while printing normal feedbacks in the right side of the screen.
    Catches key press and invoke a user defined callable object.
    """

    def __init__(self, title="Status Viewer", viewer_width=50, callback=None):
        self.buffer = deque()
        self.statuses = []
        self.onKey = callback
        self.viewer_width = viewer_width
        self.title = title
        self.running = False

        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        curses.curs_set(0)
        self.stdscr.keypad(True)
        self.stdscr.clear()
        self.stdscr.nodelay(True)

        self.maxy, self.maxx = self.stdscr.getmaxyx()
        self.maxx -= self.viewer_width + 1

    def __del__(self):
        curses.nocbreak()
        curses.echo()
        self.stdscr.keypad(True)
        curses.endwin()

    def set_onKey(self, callback):
        'Set the callback function when a key is pressed'
        self.onKey = callback

    def add_status(self, s):
        "Add a status into the list to show"
        self.statuses.append(s)

    def print(self, msg):
        "Print msg in the right side of the screen"
        rest = None
        msg = str(msg)
        if len(msg) > self.maxx:
            rest = msg[self.maxx:]
            msg = msg[:self.maxx]

        self.buffer.append(msg)
        if len(self.buffer) > self.maxy:
            self.buffer.popleft()

        if rest is not None:
            self.print(rest)
        else:
            self.update()

    def update(self):
        'Update the statuses into screen'
        stdscr = self.stdscr

        stdscr.clear()

        stdscr.addstr(0, 0, self.title, curses.A_STANDOUT)
        if len(self.statuses) != 0:
            for y, sta in enumerate(self.statuses, 1):
                stdscr.addstr(y, 0, f"{sta.name}: {sta.get()}")

        for y in range(self.maxy):
            stdscr.addch(y, self.viewer_width, '|')

        for y, msg in enumerate(self.buffer):
            stdscr.addstr(y, self.viewer_width + 1, msg)

        stdscr.refresh()

    def begin(self):
        'Begin to show status and catch key press.'
        self.running = True
        try:
            while self.running:
                c = self.stdscr.getch()
                if c == curses.ascii.ESC or c == curses.ascii.ctrl(ord('c')):
                    self.stop()
                if c != curses.ERR and self.onKey is not None:
                    self.onKey(c)
                self.update()
        except KeyboardInterrupt:
            pass

    def stop(self):
        'Stop the viewer.'
        self.running = False

    def wait_key(self):
        'Block and wait for a key press'
        self.print("Press any key to continue.")
        self.update()
        c = self.stdscr.getch()
        while c == curses.ERR:
            c = self.stdscr.getch()
