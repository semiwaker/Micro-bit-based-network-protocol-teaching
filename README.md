# Micro-bit-based-network-protocol-teaching

## Usage:
### serial_example.py
A example for teaching basic serial operations.
Require: pip install serial-interface

### socket_server_example.py
A example for teaching basic TCP operations, server side.

### socket_client_example.py
A example for teaching basic TCP operations, client side.

### status_viewer.py
This module is not expected to be understanded by the students.

A supplementary module to manipulate the console, which will create a split screen effect and acquire key presses.
The left side of the screen can show some volatile data in a fixed position, while the right side of the screen can show normal output.
Please view serial_controller.py and tcp_controller.py for further details.

If your OS is Windows, please "pip install windows-curses"

### serial_controller.py
A example of a serial controller, which works together with microbit\UART_control.py.

### serial_controller_student.py
A student version of serial controller, which works together with microbit\UART_control_student.py
Expected to distribute to the students.

### tcp_controller.py
A example of a tcp controller, which works together with microbit\wifi_control.py.

### microbit folder
All programs in this folder runs on Micro:bit.
Please load the corresponding program into Micro:bit when you run the controller programs.
