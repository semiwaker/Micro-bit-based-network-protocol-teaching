import socket

print("Please input IP && Port:")
IP, port = input().split()

print(f"IP: {IP}\nPORT: {port}")

sHost = socket.create_connection((IP, port))


while True:
    print("Please input a message:")
    try:
        s = bytes(input(), "utf-8")
    except EOFError:
        break
    sHost.sendall(s)

    try:
        s = sHost.recv(1024)
    except socket.timeout:
        print("timeout")
    except ConnectionAbortedError:
        print("connection abort")
        break
    else:
        print(f"Receive: {s}")

    if s == '':
        break


sHost.close()
