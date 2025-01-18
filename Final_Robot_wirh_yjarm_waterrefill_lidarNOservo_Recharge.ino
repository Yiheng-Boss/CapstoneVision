#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>
#include <Servo.h>

// Ultrasonic Parameters
const int trigPin = 53;
const int echoPin = 51;
int pingTravelTime;
float pingTravelDistance, cm;
float distance_threshold = 22.;

// Line following pins and variables
const int pwmPinR = 12;
const int dirPinR = 13;
const int pwmPinL = 11;
const int dirPinL = 10;
const int sensorPins[] = {A0, A1, A2, A3, A4};
int sensorValues[5];
bool isWaiting = false;
unsigned long waitStartTime = 0;
int lineCount = 0;

// LIDAR and Servo setup
TFLI2C tflI2C;
int16_t tfDist;
int16_t tfAddr = TFL_DEF_ADR;
const int16_t calibrationOffset = 4;
const int obstacleThreshold = 50; // Adjust this value based on your needs (in cm)
bool obstacleDetected = false;


// Timing constants
const unsigned long FIRST_LINE_DELAY = 8000;
const unsigned long SECOND_LINE_DELAY = 5000;
const unsigned long THIRD_LINE_DELAY = 7000;

// Pin definitions for limit switch
#define lim_X_left 45
#define lim_X_right 37
#define lim_Y_up 29
#define lim_Y_down 27

// Pin definitions for DM542 drivers
#define PUL_X 7
#define DIR_X 6
#define PUL_Y 5
#define DIR_Y 4

#define num_step 200
#define num_step_s 500

#define pump_relay  49

#define STEP_DELAY 500 // Microseconds between steps

#define WATER_SENSOR_PIN A5
#define Water_THRESHOLD_LEVEL 630.00 // Example threshold value

bool down_state = LOW;
bool up_state = LOW;
bool right_state = LOW;
bool left_state = LOW;
bool water = LOW;
bool water_PurplePlant = LOW;
bool water_SpiderPlant = LOW;
bool exit_loop = LOW;

int t1 = 1000;
int t2 = 500;
int water_delay = 0;
int second;

// Signal Pin to trigger Raspberry PI (remember to common the ground)
const int signal_pin = 31;
const int NOT_PLANT = 23;
const int PURPLE_HEART = 25;
const int SPIDER_PLANT = 24;

// ##########################
// ###### Step Up Code ######
// ##########################

// Modified setup function
void setup(){
      // Ultrasoinc sensor
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // RSP Signal Pin
    pinMode(signal_pin, OUTPUT);
    pinMode(NOT_PLANT, INPUT);
    pinMode(PURPLE_HEART, INPUT);
    pinMode(SPIDER_PLANT, INPUT);
    
    // Original pin setup
    pinMode(pwmPinR, OUTPUT);
    pinMode(dirPinR, OUTPUT);
    pinMode(pwmPinL, OUTPUT);
    pinMode(dirPinL, OUTPUT);
    for (int i = 0; i < 5; i++) {
        pinMode(sensorPins[i], INPUT);
    }

    // Limit switch setup
    pinMode(lim_X_left, INPUT);
    pinMode(lim_X_right, INPUT);
    pinMode(lim_Y_up, INPUT);
    pinMode(lim_Y_down, INPUT);

    // Pump relay setup
    pinMode(pump_relay, OUTPUT);
    digitalWrite(pump_relay, HIGH);

    // Motor pins setup
    pinMode(PUL_X, OUTPUT);
    pinMode(DIR_X, OUTPUT);
    pinMode(PUL_Y, OUTPUT);
    pinMode(DIR_Y, OUTPUT);

    // LIDAR and Servo setup
    Wire.begin();
    
    // Serial communication setup
    Serial.begin(9600);
    Serial2.begin(9600);
    Serial3.begin(9600);
}



// #######################
// ###### Loop Code ######
// #######################

// Modified loop function
void loop(){

    checkForObstacle();
    
    // Only proceed with line following if no obstacle is detected
    if (!obstacleDetected) {
        Serial.println("continue");
        Serial2.println("Robot Move...;No OBS");
        readLineSensors();
        lineFollowing();
    }

   delay(50); // Small delay between readings
}


// Function prototypes (keep all your existing function declarations)
void controlMotor(int pwmPin, int dirPin, int speed, bool direction){
    digitalWrite(dirPin, direction ? LOW : HIGH);
    analogWrite(pwmPin, speed);
}

void readLineSensors(){
    for (int i = 0; i < 5; i++) {
        sensorValues[i] = digitalRead(sensorPins[i]);
    }
}

void handleLineDetection(){
    if (sensorValues[0] == LOW && sensorValues[1] == LOW && sensorValues[2] == LOW &&
        sensorValues[3] == LOW && sensorValues[4] == LOW) {
        
        if (!isWaiting) {
            isWaiting = true;
            waitStartTime = millis();
            lineCount++;
            
            controlMotor(pwmPinL, dirPinL, 0, true);
            controlMotor(pwmPinR, dirPinR, 0, true);
        }
        
        unsigned long currentDelay;
        switch(lineCount) {
            case 1:
                Serial2.println("Robot Stop");
                currentDelay = FIRST_LINE_DELAY; 
                break;

            case 2:
                currentDelay = SECOND_LINE_DELAY;
                arm();
                break;

            case 3:
                currentDelay = SECOND_LINE_DELAY;
                arm();
                break;

            case 4:
                currentDelay = SECOND_LINE_DELAY;
                arm();
                break;

            case 5:
                controlMotor(pwmPinL, dirPinL, 0, true);
                controlMotor(pwmPinR, dirPinR, 0, true);

                double waterLevel = analogRead(WATER_SENSOR_PIN); // Read water level
                Serial.println(waterLevel); // Print for debugging
                if (waterLevel < Water_THRESHOLD_LEVEL) { 
                  Serial2.println("Refilling...;Recharging...");
                  Serial3.write('H');
                  Serial.println("Sent H & C");

                  while(waterLevel < Water_THRESHOLD_LEVEL)
                  {
                    double waterLevel = analogRead(WATER_SENSOR_PIN);
                    Serial.println(waterLevel);

                    if (waterLevel >= Water_THRESHOLD_LEVEL)
                    {
                      break;
                    }
                  }
                }
                
                Serial3.write('L');
                Serial.println("Sent L");
                Serial2.println("Refilling Done;Recharging...");

                //delay(3000);

                delay(12000);
                Serial3.write('F');
                Serial.println("Sent F");
                delay(500);
                Serial2.println("Refilling Done;Recharging Done");

                while(true) { delay(1000); }
                break;

            default:
                currentDelay = 0;
                break;
        }
        
        if (lineCount < 5 && millis() - waitStartTime >= currentDelay) {
            isWaiting = false;
            controlMotor(pwmPinL, dirPinL, 25, true);
            controlMotor(pwmPinR, dirPinR, 25, true);
            delay(500);
        }
        return;
    }
    
    if (isWaiting && sensorValues[0] == HIGH && sensorValues[1] == HIGH && 
        sensorValues[2] == HIGH && sensorValues[3] == HIGH && sensorValues[4] == HIGH) {
        isWaiting = false;
    }
}

void lineFollowing(){
    handleLineDetection();
    
    if (!isWaiting && lineCount < 5) {
        if ((sensorValues[2] == LOW && sensorValues[1] == LOW && sensorValues[3] == LOW && 
             sensorValues[4] == HIGH && sensorValues[0] == HIGH) || 
            (sensorValues[2] == LOW && sensorValues[1] == HIGH && sensorValues[0] == HIGH && 
             sensorValues[3] == HIGH && sensorValues[4] == HIGH)) {
            controlMotor(pwmPinL, dirPinL, 25, true);
            controlMotor(pwmPinR, dirPinR, 25, true);
        } 
        else if (sensorValues[2] == LOW && sensorValues[1] == LOW && sensorValues[3] == HIGH && 
                 sensorValues[4] == HIGH && sensorValues[0] == HIGH) {
            controlMotor(pwmPinR, dirPinR, 15, true);
            controlMotor(pwmPinL, dirPinL, 25, true);
        } 
        else if (sensorValues[1] == LOW && sensorValues[2] == HIGH && sensorValues[0] == HIGH && 
                 sensorValues[3] == HIGH && sensorValues[4] == HIGH) {
            controlMotor(pwmPinR, dirPinR, 7.5, true);
            controlMotor(pwmPinL, dirPinL, 25, true);
        } 
        else if ((sensorValues[0] == LOW && sensorValues[1] == HIGH && sensorValues[2] == HIGH && 
                  sensorValues[3] == HIGH && sensorValues[4] == HIGH) || 
                 (sensorValues[0] == LOW && sensorValues[1] == LOW && sensorValues[2] == HIGH && 
                  sensorValues[3] == HIGH && sensorValues[4] == HIGH) || 
                 (sensorValues[0] == LOW && sensorValues[1] == LOW && sensorValues[2] == LOW && 
                  sensorValues[3] == HIGH && sensorValues[4] == HIGH)) {
            controlMotor(pwmPinR, dirPinR, 0, true);
            controlMotor(pwmPinL, dirPinL, 20, true);
        } 
        else if (sensorValues[2] == LOW && sensorValues[3] == LOW && sensorValues[1] == HIGH && 
                 sensorValues[0] == HIGH && sensorValues[4] == HIGH) {
            controlMotor(pwmPinR, dirPinR, 35, true);
            controlMotor(pwmPinL, dirPinL, 15, true);
        }
        else if (sensorValues[0] == HIGH && sensorValues[1] == HIGH && sensorValues[2] == HIGH && 
                 sensorValues[3] == LOW && sensorValues[4] == HIGH) {
            controlMotor(pwmPinR, dirPinR, 35, true);
            controlMotor(pwmPinL, dirPinL, 7.5, true);
        } 
        else if ((sensorValues[0] == HIGH && sensorValues[1] == HIGH && sensorValues[2] == HIGH && 
                  sensorValues[3] == HIGH && sensorValues[4] == LOW) || 
                 (sensorValues[0] == HIGH && sensorValues[1] == HIGH && sensorValues[2] == HIGH && 
                  sensorValues[3] == LOW && sensorValues[4] == LOW) || 
                 (sensorValues[0] == HIGH && sensorValues[1] == HIGH && sensorValues[2] == LOW && 
                  sensorValues[3] == LOW && sensorValues[4] == LOW)) {
            controlMotor(pwmPinR, dirPinR, 35, true);
            controlMotor(pwmPinL, dirPinL, 0, true);
        } 
        else {
            controlMotor(pwmPinL, dirPinL, 0, true);
            controlMotor(pwmPinR, dirPinR, 0, true);
        }
    }
}
// [Previous controlMotor, readLineSensors, and lineFollowing functions remain exactly the same]

// #########################################
// ###### Code for Obstacle Avoidance ######
// #########################################

// New function to handle LIDAR scanning and obstacle detection
void checkForObstacle(){
     if (tflI2C.getData(tfDist, tfAddr)) {
        int16_t calibratedDist = tfDist + calibrationOffset;
        
        if (tfDist <= 0) {
            calibratedDist = 0;
        }
        
        // Print distance information
        Serial.print("Distance: ");
        Serial.print(calibratedDist);
        Serial.print(" cm / ");
        Serial.print(calibratedDist / 2.54);
        Serial.print(" inches | Status: ");
        
        // Check for obstacle
        if (calibratedDist > 0 && calibratedDist <= obstacleThreshold) {
            Serial.println("OBSTACLE DETECTED - Robot Stopped!");
            Serial2.println("OBS DETECTED");
            obstacleDetected = true;
            // Stop motors
            controlMotor(pwmPinL, dirPinL, 0, true);
            controlMotor(pwmPinR, dirPinR, 0, true);
        }
        else {
            Serial.println("Clear");
            if (obstacleDetected) {
                // Resume previous movement only if we were previously stopped by an obstacle
                obstacleDetected = false;
                if (!isWaiting && lineCount < 4) {
                    // Resume normal line following speed
                    controlMotor(pwmPinL, dirPinL, 25, true);
                    controlMotor(pwmPinR, dirPinR, 25, true);
                }
            }
        }
    } else {
        Serial.println("Error reading data from TF-Luna!");
        obstacleDetected = false;

    }
}

// ###################################
// ###### Code for Arm Movement ######
// ###################################

void initialization(){
    down_state = LOW;
    up_state = LOW;
    right_state = LOW;
    left_state = LOW;
    water = LOW;
    cm = 0;
    exit_loop = LOW;
    water_delay = 0;
    water_PurplePlant = LOW;
    water_SpiderPlant = LOW;
}

void pulse(int pin){
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);
}

void moveMotor(int pulPin, int dirPin, bool direction, int steps){
  digitalWrite(dirPin, direction); // Set direction
  for (int i = 0; i < steps; i++) {
    pulse(pulPin); // Create a pulse for each step
    delayMicroseconds(STEP_DELAY);
  }
}

void avoidMotor(int pulPin, int dirPin, bool direction, int steps){
  digitalWrite(dirPin, direction); // Set direction
  for (int i = 0; i < steps; i++) {
    pulse(pulPin); // Create a pulse for each step
    delayMicroseconds(STEP_DELAY);
  }
}

void arm_going_to_plant() {
  while(true) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(10);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin,LOW);
    pingTravelTime = pulseIn(echoPin, HIGH);
    delay(25);
    pingTravelDistance = (pingTravelTime*765.*5280.*12.)/(3600.*1000000.);
    cm = (pingTravelDistance/2.)*2.54;

    if (cm > distance_threshold) {
      // move arm down
      Serial.print("Distance: ");
      Serial.println(cm);
      String ultra_distance = "Distance:"+String(cm)+"cm";
      String comment = "Moving Down;"+ultra_distance;
      Serial2.println(comment);
      moveMotor(PUL_Y, DIR_Y, HIGH, num_step);

      if (digitalRead(lim_Y_down) == LOW) {
        avoidMotor(PUL_Y, DIR_Y, LOW, num_step_s);
        Serial.println("Limit Reached");
        Serial2.println("Limit Reached");
        Serial.println("Unable to reach desired position");
        Serial2.println("Unable to reach");
        Serial.println("Watering cancel back to home position");
        Serial2.println("Watering Cancel");
        exit_loop = HIGH;
      } 
    }
    else {
      water = HIGH;
      Serial.println("Reached");
      Serial.println("Ready to water");
      Serial2.println("Reached;Ready to water...");
      delay(2000);
      if (water_PurplePlant == HIGH){
        Serial.println("Delay = 3");
        water_delay = 3000;
      }

      if (water_SpiderPlant == HIGH){
        Serial.println("Delay = 5");
        water_delay = 5000;
      }
      break;
    }

    if (exit_loop == HIGH) {
      break;
    }
  }
  delayMicroseconds(250);
}

void arm() {
    Serial2.println("ROBOT STOP");
    delay(2000);
    initialization();
    Serial.println("Start");
    Serial2.println("EXT ARM;Moving Left");

    arm_movement();

    delay(t2);

    Serial.println("Homing");

    homing();

    Serial.println("End");
    Serial2.println("Homed");
}

void homing(){

  while (digitalRead(lim_Y_up) == HIGH) {
      Serial.println("Moving Up");
      Serial2.println("Homing...;Moving Up");
      moveMotor(PUL_Y, DIR_Y, LOW, num_step);
  }
  
  if (digitalRead(lim_Y_up) == LOW) {
    avoidMotor(PUL_Y, DIR_Y, HIGH, num_step_s);
    right_state = HIGH;
  }

  
  delay(t1);

  if (right_state == HIGH) {
    while (digitalRead(lim_X_right) == HIGH) {
      Serial.println("Moving right");
      Serial2.println("Homing...;Moving Right");
      moveMotor(PUL_X, DIR_X, HIGH, num_step); // Move X-axis right
    }
  }
  
  if (digitalRead(lim_X_right) == LOW) {
    avoidMotor(PUL_X, DIR_X, LOW, num_step_s);
  }

  delay(t1);
  Serial.println("Homed");
}

void arm_movement(){
  // Check button for Y-axis up movement
  Serial.println("Ready to move left");
  while ((digitalRead(lim_X_left) == HIGH)) {
    moveMotor(PUL_X, DIR_X, LOW, num_step);
    Serial.println("Moving left");
  }

  if (digitalRead(lim_X_left) == LOW) {
    avoidMotor(PUL_X, DIR_X, HIGH, num_step_s);
    Serial.println("Ready to capture Image");
    Serial2.println("Rdy to cap img");
    delay(2000);
  }
  
  // set a code that use to trigger PI to capture image
  digitalWrite(signal_pin, HIGH);
  delay(500);

    // wait for the feedback of PI
  while (true) {
    Serial.println("Waiting for feedback");
    Serial2.println("Wait for FB");
    delay(500);
    Serial2.println("Wait for FB.");
    delay(500);
    Serial2.println("Wait for FB..");
    delay(500);
    Serial2.println("Wait for FB...");
    delay(500);

    //if not plant
     // exit the function and set the arm to home
    if (digitalRead(NOT_PLANT) == HIGH) {
      digitalWrite(signal_pin, LOW);
      Serial.println("No plant detected");
      Serial2.println("X Plant");
      water = LOW;
      break;
    }
      
    // if it is the plant then go to a function
      // this function turn of the cam and turn on the ultrasonic sensor to measure distance
      // when the distance reached the threshold value run water function
      // exit the function and set the arm to home
    if (digitalRead(PURPLE_HEART) == HIGH){
      digitalWrite(signal_pin, LOW);
      Serial.println("PURPLE HEART PLANT detected");
      Serial2.println("/ Plant;Purple Heart");
      delay(2000);
      second = 3;
      water_PurplePlant = HIGH;
      arm_going_to_plant();
      break;
    }

    if (digitalRead(SPIDER_PLANT) == HIGH ) {
      digitalWrite(signal_pin, LOW);
      Serial.println("SPIDER PLANT detected");
      Serial2.println("/ Plant;Spider Plant");
      delay(2000);
      second = 5;
      water_SpiderPlant = HIGH;
      arm_going_to_plant();
      break;
    }
  }
    
  left_state = LOW;

  delay(t1);

  while(water) {
    for (int i = second; i > 0; i--){
      digitalWrite(pump_relay, LOW);
      Serial.println("Watering");
      String text1 = "Watering for ";
      String text2 = text1+second;
      Serial2.println(text2+";Timer:"+String(i)+"s");
      delay(1000);
    }
    digitalWrite(pump_relay, HIGH);
    Serial.println("Watering Done");
    Serial2.println("Watering Done");
    delay(2000);
    break;
  }
}