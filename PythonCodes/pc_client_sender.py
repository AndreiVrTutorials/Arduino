# pc_client_sender.py
import socket

RPI_IP = '192.168.1.100'  #
PORT = 65432

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((RPI_IP, PORT))
    print(f"Connected to Raspberry Pi at {RPI_IP}:{PORT}")
    while True:
        msg = input("Send message to ATtiny: ")
        if msg.lower() == "exit":
            print("Closing connection.")
            break
        s.sendall((msg + '\n').encode())