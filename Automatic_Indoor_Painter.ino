// Author : Roshan Jude Cardoza
// Version : 1.0
// Date Modified : 16.12.2024
// Project Title : Automating Indoor Wall Painting



#include <Servo.h>

#define DipServopin A0 // Servo to dip Paint

// Motors controlling Front and Back motion of the paint brush/roller for either avoidance of objects such as
// Light Switches or to perform Dipping Action in Paint
#define DipMotorFront A2 
#define DipMotorBack A1

// Ultrasonic Distance Sensors Pin Initializations
#define trigleft 2
#define trigfront 3
#define trigright 4

#define echoleft 11
#define echofront 12
#define echoright 13

// Robot Movement Motor Pins
#define motorRplus 8  // Right motor forward
#define motorRminus 7 // Right motor backward
#define motorLplus 6  // Left motor forward
#define motorLminus 5 // Left motor backward

// Paint brush/roller up and down motors that are used for painting
#define motorup 9
#define motordown 10

const int triglow = 2; // Small delay for ultrasonic trigger stabilization

bool cornerDetected = false; // Flag to track corner detection

Servo DipServo; 

#define irsensor A3 // To check if paint level is low

void setup()
{
  // Ultrasonic distance sensors
  pinMode(trigleft, OUTPUT);
  pinMode(trigfront, OUTPUT);
  pinMode(trigright, OUTPUT);

  pinMode(echoleft, INPUT);
  pinMode(echofront, INPUT);
  pinMode(echoright, INPUT);

  // Robot drive motors
  pinMode(motorRplus, OUTPUT);
  pinMode(motorRminus, OUTPUT);
  pinMode(motorLplus, OUTPUT);
  pinMode(motorLminus, OUTPUT);

  // Vertical brush motors
  pinMode(motorup, OUTPUT);
  pinMode(motordown, OUTPUT);

  // Brush dipping servo
  DipServo.attach(DipServopin);

  // Brush retract/extend motors
  pinMode(DipMotorFront, OUTPUT);
  pinMode(DipMotorBack, OUTPUT);

  // Baud rate
  Serial.begin(9600);
}

// Basic movement functions
void forward()
{
  digitalWrite(motorRplus, HIGH);
  digitalWrite(motorRminus, LOW);
  digitalWrite(motorLplus, HIGH);
  digitalWrite(motorLminus, LOW);
  delay(1000); // Increased delay to 1000ms for observable movement (adjustable)
}

void turnleft()
{
  digitalWrite(motorRplus, HIGH);
  digitalWrite(motorRminus, LOW);
  digitalWrite(motorLplus, LOW);
  digitalWrite(motorLminus, LOW);
  delay(1500); // Increased delay to 1500ms for a full turn (adjustable)
}

void turnright()
{
  digitalWrite(motorRplus, LOW);
  digitalWrite(motorRminus, LOW);
  digitalWrite(motorLplus, HIGH);
  digitalWrite(motorLminus, LOW);
  delay(1500); // Increased delay to 1500ms for a full turn (adjustable)
}

void backward()
{
  digitalWrite(motorRplus, LOW);
  digitalWrite(motorRminus, HIGH);
  digitalWrite(motorLplus, LOW);
  digitalWrite(motorLminus, HIGH);
  delay(1000); // Increased delay to 1000ms for observable movement (adjustable)
}

void stop()
{
  digitalWrite(motorRplus, LOW);
  digitalWrite(motorRminus, LOW);
  digitalWrite(motorLplus, LOW);
  digitalWrite(motorLminus, LOW);
  delay(1000); // Added delay to ensure motors have stopped (adjustable)
}

// Ultrasonic sensor functions
long durationSensor(int trigpin, int echopin)
{
  digitalWrite(trigpin, LOW);
  delay(triglow);
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin, LOW);
  return pulseIn(echopin, HIGH) / 2;
}

float distance(int trigpin, int echopin)
{
  long time = durationSensor(trigpin, echopin);
  return time * 0.034; // Adjust if needed based on sensor calibration
}

// Dipping Function : First the brush is moved out of the wall. The Servo is turned so the brush is facing
// Downwards towards the paint canister. The brush is then extended towards the paint. After a delay, the 
// brush is retracted and made parallel to the ground. Lastly, the brush is extended towards the wall

void dipbrush()
{
  // Retract brush
  digitalWrite(DipMotorBack, HIGH);
  delay(2000); 
  digitalWrite(DipMotorBack, LOW);
  
  // Make brush face the paint canister
  DipServo.write(90);
  delay(2000); 
  
  // Dip brush
  digitalWrite(motordown, HIGH);
  delay(2000); 
  digitalWrite(motordown, LOW);
  
  // Wait for brush to be submerged for a short while
  delay(3000); // Increased delay to 3000ms to allow paint to adhere
  
  // Bring brush out of paint
  digitalWrite(motorup, HIGH);
  delay(2000); 
  digitalWrite(motorup, LOW);
  
  // Align with wall
  DipServo.write(0);
  delay(2000); 
  
  // Extend brush to wall
  digitalWrite(DipMotorFront, HIGH);
  delay(2000); 
  digitalWrite(DipMotorFront, LOW);
}

// Extra function to enable extension of brush (Made parallel to ground)
void extendBrush()
{
  Serial.println("Extending BRUSH\n");
  digitalWrite(DipMotorFront, HIGH);
  delay(1000); 
  DipServo.write(0);
  delay(1000); 
  digitalWrite(DipMotorFront, LOW);
  delay(1000); 
}

// Extra function to enable retraction of brush (Made perpendicular to ground)

void contractBrush()
{
  digitalWrite(DipMotorBack, HIGH);
  delay(2000); 
  DipServo.write(90);
  delay(2000); 
  digitalWrite(DipMotorBack, LOW);
  delay(2000); 
}


// Paint one wall until no wall is detected
// If a light switch (frontDist < thresholdfront) is encountered, pause painting until it is clear.

bool paintwall()
{
  int thresholdfront = 25;    // Threshold to detect a light switch or close obstacle (adjustable) in cm
  int wallDetectDist = 30;    // If distance > wallDetectDist, consider it the end of the wall in cm
  int heightofwall = 240;     // Total height of the wall in cm
  int stepHeight = 24;        // Incremental height step in cm (This can be changed depending on the brush)
  int currentHeight = 30;     // Current brush height in cm
  bool paintingUp = true;     // Direction: true = up, false = down
  int bottomHeight = 30;      // Bottom of wall assuming there is skirting done on tiles in cm
  int doordistancemax = 30;   // Door detection distance in cm

  // Move up from bottom to top once
  while (paintingUp) 
  {
    // Calculate Front and Right Ultrasonic Sensor Distances
    float frontDist = distance(trigfront, echofront);
    float rightDist = distance(trigright, echoright);

    // If an obstacle is detected in front, wait until it's clear
    while (frontDist < thresholdfront)
    {
      digitalWrite(motorup, LOW);
      digitalWrite(motordown, LOW);

      // Check distance again after waiting
      frontDist = distance(trigfront, echofront);
      delay(1000); 
      currentHeight +=stepHeight; // Adding step height to account for height of wall
      Serial.print("WALL OBJECT DETECTED. IGNORING FOR PAINTING\n");
      if (currentHeight >= heightofwall)
      {
        break;
      }
    }

    // If door is detected, Motor with brush moves upwards till the Ultrasonic sensor detects 30cm 

    // Flag so that the brush does not infinitely retract
    int brushdoormovementup = 1; 

    if (frontDist > doordistancemax)
    {

      Serial.println("DOOR DETECTED");
        if(brushdoormovementup == 1)
        {
        digitalWrite(DipMotorBack,HIGH); // Moving brush out of the way
        delay(100);
        digitalWrite(DipMotorBack,LOW);
        }
      brushdoormovementup++;
      digitalWrite(motorup,HIGH);
      currentHeight += stepHeight;
      if(frontDist == doordistancemax)
      {
        digitalWrite(DipMotorFront,HIGH); // Placing brush back in path
        delay(100);
        digitalWrite(DipMotorFront,LOW);
        break;
      }      
    }

    // Move up if not at the top yet
    if (currentHeight < heightofwall)
    {
      // Message to signal painting direction
      Serial.print("PAINTING WALL UPWARDS\n");
      currentHeight += stepHeight;
      digitalWrite(motorup, HIGH);
      delay(1500); 
      digitalWrite(motorup, LOW);

    }
    else
    {
      // Reached the top, start going down
      paintingUp = false;
    }
  }

  // Move down from top to bottom once
  while (!paintingUp) 
  {
    // Calculating object distance again
    float frontDist = distance(trigfront, echofront);

    // If an obstacle is detected, wait until it's clear
    while (frontDist < thresholdfront)
    {
      digitalWrite(motorup, LOW);
      digitalWrite(motordown, LOW);

      // Check distance again after waiting
      frontDist = distance(trigfront, echofront);
      delay(1000); 
      currentHeight -= stepHeight;
      Serial.print("WALL OBJECT DETECTED. IGNORING FOR PAINTING\n");
      if (currentHeight <= bottomHeight)
      {
        break;
      }
      
    }

    
    // If door is detected, Motor with brush moves downwards till the Ultrasonic sensor detects 30cm 
    
    // Flag so that brush does not infinitely retract
    int brushdoormovementdown = 1;

    if (frontDist > doordistancemax)
    {

      Serial.println("DOOR DETECTED");
        if(brushdoormovementdown == 1)
        {
        digitalWrite(DipMotorBack,HIGH);
        delay(100);
        digitalWrite(DipMotorBack,LOW);
        }

      digitalWrite(motorup,HIGH);
      currentHeight -= stepHeight;
      brushdoormovementdown++;
      if(frontDist == doordistancemax)
      {
        digitalWrite(DipMotorFront,HIGH);
        delay(100);
        digitalWrite(DipMotorFront,LOW);
        break;
      }      
    }

    // Move down if not at the bottom yet
    if (currentHeight > bottomHeight)
    {
      // Message to signal painting direction
      Serial.print("PAINTING WALL DOWNWARDS\n");      
      currentHeight -= stepHeight;
      digitalWrite(motordown, HIGH);
      delay(1500); 
      digitalWrite(motordown, LOW);

      Serial.print(" ");
    }
    else
    {
      // Reached the bottom again, painting cycle completed
      Serial.println("Completed one up and down stroke of the wall.\n");

      // Dipping brush in Paint to ensure even coating between sections
      Serial.print("DIPPING BRUSH \n");      
      dipbrush();
      delay(3000); 

      return true;
    }
  }

  // Just a fallback return for debugging
  return true;
}



// Always find the left corner of room / wall
void findCorner()
{
  float frontDist, leftDist, rightDist;

  while (!cornerDetected)
  {
    // Read distances from ultrasonic sensors
    frontDist = distance(trigfront, echofront);
    leftDist = distance(trigleft, echoleft);
    rightDist = distance(trigright, echoright);

    // Debugging: Print sensor readings
    Serial.print("Front Distance: ");
    Serial.print(frontDist);
    Serial.print(" cm, Left Distance: ");
    Serial.print(leftDist);
    Serial.print(" cm, Right Distance: ");
    Serial.print(rightDist);
    Serial.println(" cm");

    // Check for a left corner: front and right blocked
    if (frontDist < 30 && rightDist < 30)
    {
      stop(); // Stop all motors
      delay(2000); 
      turnright();
      delay(2000); 
      stop();
      Serial.println("Left corner detected!");
      cornerDetected = true; // Set the flag to exit the loop
      break;
    }
    else
    {
      // Behavior Based on Front Distance
      if (frontDist >= 30)
      {
        // If the front is clear, move forward
        forward();
        Serial.println("Moving forward.");
      }
      else
      {
        // If the front is blocked, turn left to align alongside the right wall
        turnleft();
        Serial.println("Turning left to align with the wall.");
        delay(2000); 

        // Ensuring proper alignment alongside the left wall
        forward();
        delay(1500); 
        stop(); // Briefly stop to maintain alignment
        Serial.println("Aligning alongside the wall.");
      }
    }

    delay(1500); // Ensuring proper working and enough time before starting painting
  }

  Serial.println("Corner detection complete. Ready for the next operation.");
}

// Move to the next wall after finishing painting the current one
void moveToNextWall()
{
  Serial.println("Moving to the next wall...");

  // Step 1: Back up a bit
  backward();
  delay(2000); 
  stop();

  // Step 2: Turn right (since we start with left corner) to face the next wall plane
  turnright();
  delay(2000); 

  // Step 3: Move back further so that all sensors have enough headway
  backward();
  delay(2000); 

  // Stop and wait for corner detection
  stop();
  delay(1000);

  // Corner detection on new wall the robot is facing
  findCorner();
  Serial.print("NEXT WALL FOUND");
}

void movetonextsection()
{
  Serial.print("Moving on to NEXT Section\n");

  // Finding Front Distance
  float frontDist = distance(trigfront,echofront);

  // Global variable just in case we want to implement dynamic wall painting
  static int sectionCount = 0;
  sectionCount++;

  // Back up a bit
  backward();
  delay(3000);

  // Turn right since we will always be more LEFT than RIGHT
  turnright();
  delay(2000);

  // Move a bit towards the next right section
  forward();
  delay(2000);

  // Align with wall
  turnleft();
  delay(2000);

  // Move forward till we reach the wall
  forward();
  delay(3000);

  // Stop to begin painting
  stop();
  delay(2000);
}

// Main LOOP function
void loop()
{
  static int wallcount = 0; // To count the number of walls already painted
  const int maxwallcount = 4; // Maximum number of walls that exist in room

  // Finding end of wall section
  float rightDist = distance(trigright,echoright);
  const int wallDetectDist = 30;

  // IR Sensor readings 
  int sensor = 1; // Temporary Variable used to trigger the IR sensor. In reality, this would be automatically read and triggered but TinkerCAD does not allow for changes to the IR sensor
  // if(analogRead(irsensor)<300) //For Normal operation
  
  // Make sure the brush is always facing the ground to avoid splatter when finding corner
  contractBrush();
  delay(1000);

  // Checking if paint level is HIGH
  if(sensor ==1) // No IR light detected OR Paint is present
  {
    // If we haven't found a corner yet, find it first
    if (!cornerDetected)
    {
      findCorner();

      // Extend the brush for painting once the corner is found
      extendBrush();
      delay(2000);
    }
    while (wallcount < maxwallcount) // Ensuring that only 4 walls will be painted
    {
      // Checking right sensor distance
      rightDist = distance(trigright,echoright);

      if (rightDist > wallDetectDist) // As long as the right sensor is detecting wall far away
      {
        paintwall();
        movetonextsection();
        
        //Check if we've reached the end of the wall
        rightDist = distance(trigright,echoright);
        
        if (rightDist <= wallDetectDist) // If we have reached the end of the wall
        {
          Serial.println("End of wall detected. This wall completed");
          moveToNextWall();
          wallcount++;
          Serial.print("\n");
          Serial.print("Number of Walls Painted = ");
          Serial.print(wallcount);
          Serial.print("\n");
          
        }
      }
    }
    if(wallcount == maxwallcount)
    {
      // Message to indicate all walls have been printed and all tasks have been completed in the room by the robot.
      Serial.println("ALL WALLS HAVE BEEN PAINTED IN ROOM\n");
      return;
    }
  }
  else
  {
    // If paint is low, robot is in low energy state until the paint is refilled
    Serial.println("PAINT LEVEL LOW!\n");
    return;
  }

}