import serial
import time
import os

# ================= CONFIGURATION =================
SERIAL_PORT = 'COM8'   
BAUD_RATE = 115200
# =================================================

def main():
    photo_count = 1
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=50)
        print(f"Connected to {SERIAL_PORT}")
    except Exception as e:
        print(f"Could not open port: {e}")
        return

    ser.reset_input_buffer()
    time.sleep(2) 

    print("Press 'p' + ENTER to capture (or 'q' to quit)...")

    while True:
        user_input = input("Command: ").lower()
        if user_input == 'q':
            break
        if user_input != 'p':
            continue
        
        print("Triggering Camera...")
        ser.write(b'p') 

        while True:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if line.startswith("IMG_START:"):
                    image_len = int(line.split(":")[1])
                    print(f"Receiving Photo {photo_count} ({image_len} bytes)...")
                    
                    image_data = ser.read(image_len)
                    
                    if len(image_data) == image_len:
                        filename = f"photo{photo_count}.jpg"
                        with open(filename, "wb") as f:
                            f.write(image_data)
                        print(f"Success! Saved to {filename}")
                        photo_count += 1
                    else:
                        print(f"Error: Got {len(image_data)} of {image_len} bytes.")
                    
                    break # Back to user input
                
                elif line:
                    print(f"Arduino: {line}")

            except Exception as e:
                print(f"Read error: {e}")
                break

    ser.close()
    print("Closed connection.")

if __name__ == "__main__":
    main()
