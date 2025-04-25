#include <SPI.h>
#include <RH_RF69.h>

// Singleton instance of the radio driver
RH_RF69 rf69;

const int joyX = A0;  // Joystick X-axis (turning)
const int joyY = A1;  // Joystick Y-axis (forward/backward)

int xValue, yValue;
int deadZone = 10;  // Define the dead zone range

void setup() 
{
  // Initialize Serial Monitor (for debugging)
  Serial.begin(9600);
  while (!Serial) 
    ;
  if (!rf69.init())
    Serial.println("init failed");

  // Set frequency
  if (!rf69.setFrequency(433.0))
    Serial.println("setFrequency failed");

  rf69.setTxPower(14, true);

  // Set encryption key
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
}


void loop()
{
  // Read joystick values
  xValue = analogRead(joyX);
  yValue = analogRead(joyY);

  // Map joystick Y-axis values to motor speed (0 to 255 for PWM range)
  int forwardSpeed = map(yValue, 0, 1023, -255, 255); // Forward/backward speed
  int turnSpeed = map(xValue, 0, 1023, -255, 255);    // Turning speed

  // Apply dead zones for both X and Y axes
  if (abs(yValue - 512) < deadZone) {
    forwardSpeed = 0;
  }
  if (abs(xValue - 512) < deadZone) {
    turnSpeed = 0;
  }

  Serial.println("Sending to rf69_server");
  // Send a message to rf69_server
  uint8_t data[4];
  data[0] = forwardSpeed & 0xFF;          // Send the lower byte of forward speed
  data[1] = (forwardSpeed >> 8) & 0xFF;   // Send the higher byte of forward speed
  data[2] = turnSpeed & 0xFF;             // Send the lower byte of turn speed
  data[3] = (turnSpeed >> 8) & 0xFF;      // Send the higher byte of turn speed

  rf69.send(data, sizeof(data));
  
  rf69.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf69.waitAvailableTimeout(500))
  { 
    // Should be a reply message for us now   
    if (rf69.recv(buf, &len))
    {
      Serial.print("got reply: ");
      Serial.println((char*)buf);
    }
    else
    {
      Serial.println("recv failed");
    }
  }
  else
  {
    Serial.println("No reply, is rf69_server running?");
  }
  delay(50);
}

