#include <Arduino.h>
#include <Servo.h>

// Pin definitions for servos
const int SERVO_PINS[4] = {9, 10, 11, 12}; // Adjust pins as needed

Servo servos[4]; // Array to hold servo objects
String inputBuffer = ""; // Buffer for incoming serial data

// Global position tracking for axes
float currentX = 0;
float currentY = 0;
float currentZ = 0;

struct ServoData {
  int ids[4];
  int angles[4];
  bool valid[4];
};

struct GCodeData {
  float x;
  float y;
  float z;
};

// put function declarations here:
void moveServo(int servoId, int angle);
void moveServos(ServoData data);
ServoData parseServoString(String input);
ServoData inversekinematics(float x, float y, float z);
GCodeData parseGCode(String input);

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Robot Arm Controller Ready");
  Serial.println("Send commands like: a90 b45 c120 or a90b45c120");
  
  // Attach servos to pins
  for (int i = 0; i < 4; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(90); // Initialize to middle position
  }
  
  inputBuffer.reserve(100); // Reserve space for input buffer
}

void loop() {
  // Check if serial data is available
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    
    // If newline character, process the command
    if (inChar == '\n' || inChar == '\r') {
      if (inputBuffer.length() > 0) {
        // Echo the received command
        Serial.print("Received: ");
        Serial.println(inputBuffer);
        
        // Parse and execute the servo command
        ServoData data = parseServoString(inputBuffer);
        moveServos(data);
        
        Serial.println("Command executed");
        
        // Clear the buffer
        inputBuffer = "";
      }
    } else {
      // Add character to buffer
      inputBuffer += inChar;
    }
  }
}

// put function definitions here:
void moveServo(int servoId, int angle) {
  // Move servo with servoId to the specified angle
  servos[servoId].write(angle);
}

void moveServos(ServoData data) {
  // Move only the servos that were included in the parsed string
  for (int i = 0; i < 4; i++) {
    if (data.valid[i]) {
      moveServo(data.ids[i], data.angles[i]);
    }
  }
}

ServoData parseServoString(String input) {
  // Parse string like "a90b45c120" or "a90 b45 c120" (G-code style)
  ServoData data;
  
  // Initialize all as invalid
  for (int i = 0; i < 4; i++) {
    data.valid[i] = false;
  }
  
  input.trim(); // Remove leading/trailing whitespace
  input.toUpperCase(); // Make case-insensitive
  
  int count = 0;
  int i = 0;
  
  while (i < input.length() && count < 4) {
    char c = input.charAt(i);
    
    // Skip spaces and commas
    if (c == ' ' || c == ',') {
      i++;
      continue;
    }
    
    // Check if current character is a letter (A-D)
    if (c >= 'A' && c <= 'D') {
      char servoLetter = c;
      int servoId = servoLetter - 'A'; // Convert A->0, B->1, C->2, D->3
      i++; // Move past the letter
      
      // Extract the number following the letter
      String numStr = "";
      while (i < input.length()) {
        char digit = input.charAt(i);
        if (digit >= '0' && digit <= '9') {
          numStr += digit;
          i++;
        } else {
          break; // Stop at next letter, space, or any non-digit
        }
      }
      
      // If we got a number, process it
      if (numStr.length() > 0) {
        int angle = numStr.toInt();
        
        // Validate angle
        if (angle >= 0 && angle <= 180) {
          data.ids[count] = servoId;
          data.angles[count] = angle;
          data.valid[count] = true;
          count++;
        }
      }
    } else {
      // Skip any other character
      i++;
    }
  }
  
  return data;
}

ServoData inversekinematics(float x, float y, float z) {
  // Placeholder inverse kinematics function
  // This should compute servo angles based on x, y, z coordinates
  ServoData data;
  float angle0=0;
  float angle1=0;
  float angle2=0;







  // For demonstration, just map x, y, z to servo angles linearly
  data.ids[0] = 0; data.angles[0] = angle0; data.valid[0] = true;
  data.ids[1] = 1; data.angles[1] = angle1; data.valid[1] = true;
  data.ids[2] = 2; data.angles[2] = angle2; data.valid[2] = true;
  data.ids[3] = 3; data.angles[3] = 0; data.valid[3] = false; // Fixed angle for servo D
  
  return data;
}

GCodeData parseGCode(String input) {
  // Parse GCode-like string "x10 y26 z45"
  GCodeData data;
  data.x = currentX; // Start with current position
  data.y = currentY;
  data.z = currentZ;
  input.toUpperCase();
  
  int xPos = input.indexOf('X');
  int yPos = input.indexOf('Y');
  int zPos = input.indexOf('Z');
  
  if (xPos != -1) data.x = input.substring(xPos + 1).toFloat();
  if (yPos != -1) data.y = input.substring(yPos + 1).toFloat();
  if (zPos != -1) data.z = input.substring(zPos + 1).toFloat();
  
  // Update global position
  currentX = data.x;
  currentY = data.y;
  currentZ = data.z;
  
  return data;
}

