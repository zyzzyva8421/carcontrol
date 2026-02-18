import cv2
import socket
import struct
import pickle

HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 9999  # Port to listen on

def main():
    # Open the first video device (e.g., /dev/video0)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open video device.")
        return

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)
    print(f"Server listening on {HOST}:{PORT}")

    conn, addr = server_socket.accept()
    print(f"Connection from: {addr}")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # Serialize the frame
        data = pickle.dumps(frame)
        message_size = struct.pack("Q", len(data))

        # Send message size and data
        conn.sendall(message_size + data)

    cap.release()
    conn.close()
    server_socket.close()

if __name__ == "__main__":
    main()