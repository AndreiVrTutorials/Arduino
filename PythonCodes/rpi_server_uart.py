# rpi_server_uart.py 
import socket
import threading
import serial

HOST = '0.0.0.0'
PORT = 65432

# UART hacia ATtiny
uart = serial.Serial('/dev/serial1', 9600, timeout=1)

def handle_client(conn, addr):
    print(f"[Connected by {addr}]")
    try:
        while True:
            print("Waiting messages...")
            data = conn.recv(1024)
            print(f"Raw data received: {data}")  # Debug print
            if not data:
                print("No data received, breaking loop.")
                break
            message = data.decode().strip()
            print(f"Received from PC: {message}")

            # Enviar al ATtiny por UART
            bytes_written = uart.write((message + '\n').encode())
            if bytes_written > 0:
                print(f"Sent to ATtiny: {message}")
            else:
                print("Failed to send message to ATtiny")

            # Leer respuesta del ATtiny (si hay)
            if uart.in_waiting:
                response = uart.readline().decode().strip()
                print(f"ATtiny replied: {response}")

    except ConnectionResetError:
        print("Client disconnected")
    finally:
        conn.close()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"[Raspberry Pi Server listening on {HOST}:{PORT}]")
    while True:
        conn, addr = s.accept()
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
