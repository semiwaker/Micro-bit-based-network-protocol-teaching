import serial
import time


def write(ser, msg):
    # Make sure the whole message is sent.
    total_sent = 0
    while total_sent < len(msg):
        sent = ser.write(msg[total_sent:])
        if sent == 0:
            raise serial.SerialException()
        total_sent += sent


def read(ser, byte_cnt):
    # Make sure the whole message is read.
    chunks = []
    while byte_cnt > 0:
        chunk = ser.read(byte_cnt)
        if len(chunk) == 0:
            raise serial.SerialException()
        chunks.append(chunk)
        byte_cnt -= len(chunk)
    return b"".join(chunks)


print("Please input serial name:")
s = input()

ser = serial.Serial(s, 115200, timeout=0.5)

while True:
    s = input() + "\r\n"
    print("written: %s" % bytes(s, "utf-8"))

    try:
        write(ser, bytes(s, "utf-8"))

        # Sleep so that the whole message can be received.
        time.sleep(0.1)

        s = read(ser.inWaiting())
        print(s)
        print()
    except serial.SerialException:
        print("Disconnected")
        break
    except serial.SerialTimeoutException:
        print("Timeout")
