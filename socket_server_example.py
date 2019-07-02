import socket

sServer = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)

host_name = socket.gethostname()
IP = socket.gethostbyname(host_name)
port = 6666
print(f"IP: {IP}\nPORT: {port}")

sServer.bind((IP, port))

sServer.listen()

sClient, client_address = sServer.accept()

sClient.settimeout(3)

while True:
    print("Please input a message:")
    try:
        s = bytes(input(), "utf-8")
    except EOFError:
        break
    sClient.sendall(s)

    try:
        s = sClient.recv(1024)
    except socket.timeout:
        print("timeout")
    except ConnectionAbortedError:
        print("connection abort")
        break
    else:
        print(f"Receive: {s}")

    if s == '':
        break


sServer.close()
