# Import the required libraries
import cv2
import asyncio
import websockets
import pickle
import zlib

# Define the IP address and port for the server
IP = "127.0.0.1"
PORT = 8080

# Define a function to receive and display frames from the server
async def receive_frame():
    # Create a WebSocket connection to the server
    async with websockets.connect(f"ws://{IP}:{PORT}") as websocket:
        # Loop until the connection is closed
        while True:
            # Receive data from the server
            data = await websocket.recv()
            # Deserialize the data using pickle
            compressed = pickle.loads(data)
            # Decompress the data using zlib
            buffer = zlib.decompress(compressed)
            # Decode the data as JPEG
            frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
            # Display the frame in a window
            cv2.imshow("Frame", frame)
            print(f"receive {frame.shape}")
            # Wait for a key press or exit if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    # Destroy all windows
    cv2.destroyAllWindows()

# Run the client until interrupted
asyncio.get_event_loop().run_until_complete(receive_frame())