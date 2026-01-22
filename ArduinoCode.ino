#include <Arduino.h>

#define CAM_RX_PIN  18  
#define CAM_TX_PIN  17  

HardwareSerial camSerial(1); 

const byte TAKE_PICTURE[] = {0x56, 0x00, 0x36, 0x01, 0x00};
const byte GET_SIZE[]     = {0x56, 0x00, 0x34, 0x01, 0x00};
const byte RESUME_FRAME[] = {0x56, 0x00, 0x36, 0x01, 0x03};
const byte SET_BAUD_1152[] = {0x56, 0x00, 0x24, 0x03, 0x01, 0x0D, 0xA6};

void clearBuffer() { while (camSerial.available()) camSerial.read(); }

bool waitForBytes(int count) {
  unsigned long start = millis();
  while (camSerial.available() < count) {
    if (millis() - start > 3000) return false;
  }
  return true;
}

void setup() {
  // Use a slower USB baud rate temporarily for better stability during testing
  Serial.begin(115200); 
  
  // Start camera at default 38400
  camSerial.begin(38400, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
  delay(2000); // Wait for power to stabilize

  // Attempt baud rate change
  camSerial.write(SET_BAUD_1152, sizeof(SET_BAUD_1152));
  delay(500);
  camSerial.end();

  // Restart at 115200
  camSerial.begin(115200, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
  delay(1000);
  
  clearBuffer();
  Serial.println("READY_FOR_COMMAND"); 
}

void loop() {
  if (Serial.available()) {
    char key = Serial.read();
    if (key == 'p' || key == 'P') {
      Serial.println("Starting_Capture...");
      clearBuffer();
      
      camSerial.write(TAKE_PICTURE, sizeof(TAKE_PICTURE));
      if (!waitForBytes(5)) {
        Serial.println("Error: Camera Timeout (Capture)");
        return;
      }
      clearBuffer();

      camSerial.write(GET_SIZE, sizeof(GET_SIZE));
      if (!waitForBytes(9)) {
        Serial.println("Error: Camera Timeout (Size)");
        return;
      }
      
      byte resp[9];
      for(int i=0; i<9; i++) resp[i] = camSerial.read();
      uint32_t jpgLen = (resp[5] << 24) | (resp[6] << 16) | (resp[7] << 8) | resp[8];

      Serial.print("IMG_START:");
      Serial.println(jpgLen);

      uint32_t addr = 0;
      while (addr < jpgLen) {
        uint16_t bytesToRead = min((uint32_t)32, jpgLen - addr); // Smaller chunks for power stability
        byte readCmd[] = {
          0x56, 0x00, 0x32, 0x0C, 0x00, 0x0A, 
          0x00, 0x00, (byte)(addr >> 8), (byte)(addr & 0xFF), 
          0x00, 0x00, (byte)(bytesToRead >> 8), (byte)(bytesToRead & 0xFF), 
          0x00, 0x0A
        };

        camSerial.write(readCmd, sizeof(readCmd));
        if (waitForBytes(5 + bytesToRead + 5)) {
          for(int i=0; i<5; i++) camSerial.read(); 
          for(int i=0; i<bytesToRead; i++) Serial.write(camSerial.read()); 
          for(int i=0; i<5; i++) camSerial.read(); 
          addr += bytesToRead;
        } else {
          Serial.println("Error: Chunk Timeout");
          break;
        }
      }
      camSerial.write(RESUME_FRAME, sizeof(RESUME_FRAME));
      Serial.println("Capture_Finished");
    }
  }
}
