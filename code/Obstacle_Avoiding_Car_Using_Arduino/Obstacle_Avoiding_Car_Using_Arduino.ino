//Arduino Obstacle Avoiding Car Code
//by Technical Accent 


#include <Servo.h> 

// Ultrasonic sensor pins
#define trig 2
#define echo 4

// RGB led pins 
#define LR 9       // Led Right
#define LC 13     // Led Center
#define LL 10     // Led Left

#define LRR A3   // Led Right Red
#define LCR A2   // Led Center Red
#define LLR A1  // Led Left Red

// Motor control pins
#define LEFT_MOTOR_PIN1 8
#define LEFT_MOTOR_PIN2 7
#define RIGHT_MOTOR_PIN1 12
#define RIGHT_MOTOR_PIN2 11

#define ENA 6 // Enable A pin for motor speed control
#define ENB 3 // Enable B pin for motor speed control

// Distance thresholds for obstacle detection
//#define MAX_DISTANCE 80
#define MIN_DISTANCE_BACK 12

//float timeOut = 2*(MAX_DISTANCE+10)/100/340*1000000;
// Maximum and minimum motor speeds
#define MAX_SPEED 100
#define MIN_SPEED 75

Servo servoLook;

void setup() {
  // Set motor control pins as outputs
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(LR, OUTPUT);
  pinMode(LC, OUTPUT);
  pinMode(LL, OUTPUT);

  pinMode(LRR, OUTPUT);
  pinMode(LCR, OUTPUT);
  pinMode(LLR, OUTPUT);

  digitalWrite(LR, LOW);
  digitalWrite(LC, LOW);
  digitalWrite(LL, LOW);

  analogWrite(LRR, 0);
  analogWrite(LCR, 0);
  analogWrite(LLR, 0);

  servoLook.attach(5);
  //Set the Trig pins as output pins
  pinMode(trig, OUTPUT);
  
  //Set the Echo pins as input pins
  pinMode(echo, INPUT);

  analogWrite(ENA, 53.5); // speed for motor A  0-LOW speed, 255-Full speed
  analogWrite(ENB, 45); // speed for motor B
  
  // Initialize the serial communication for debugging
  Serial.begin(9600);
}

void loop() {

  servoLook.write(90);
  delay(750);
  int distance = getDistance();
  if (distance >= MIN_DISTANCE_BACK) {
    
    moveForward();
   
    Serial.println("forward");
  } 
   while(distance >= MIN_DISTANCE_BACK)
   {
    distance = getDistance();
    delay(50);
   }
   Stop(); 
   int turnDir = checkDirection();
   Serial.println(turnDir);
   switch(turnDir)
   {
    case 0:                                       //Turn left
    Serial.println("Left");
      turnLeft();
      delay(425);
      Stop();
      break;
    
    case 2:                                       //Turn right
    Serial.println("Right");
      turnRight();
      delay(415);
      Stop();
      break;
      
    case 3:                                       //move forward
    Serial.println("Forward");
     moveForward();
      break;
    }
}

int getDistance()                                   //Measure the distance to an object
{
  unsigned long pulseTime;                          //Create a variable to store the pulse travel time
  int distance;                                     //Create a variable to store the calculated distance
  digitalWrite(trig, HIGH);                         //Generate a 10 microsecond pulse
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  pulseTime = pulseIn(echo, HIGH);         //Measure the time for the pulse to return
  distance  = pulseTime / 29 / 2;         //Calculate the object distance based on the pulse time
  Serial.println(distance);
  return distance;
}

int checkDirection()                                            //Check the left and right directions and decide which way to turn
{
  int distances [3] = {0,0,0};                                    //Left ,right and Front distances
  int turnDir;                                               //Direction to turn, 0 left, 1 turn around, 2 right, 3 forward
  servoLook.write(180);                                         //Turn servo to look left
  delay(500);
  distances [0] = getDistance();                                //Get the left object distance

  servoLook.write(90);                                         //Turn servo to look left
  delay(500);
  distances [3] = getDistance();
  servoLook.write(0);                                           //Turn servo to look right
  delay(1000);
  distances [1] = getDistance();                                //Get the right object distance
  if (distances[0]<=25 && distances[1]<=25)                   //If both directions are blocked, move forward
    turnDir = 3;                                             
  else if (distances[0]>=distances[1])                          //If left has more space, turn left
    turnDir = 0;  //left
  else if (distances[0]<distances[1])                           //If right has more space, turn right
    turnDir = 2;  //right
   //else if (distances[0]<=50 && distances[1]<=50 && distances[3]<=50)    //If All directions are blocked, turn around
    //turnDir = 1;  
  return turnDir;
}

// Motor control functions
void moveForward() {
 
  analogWrite(ENA, 60.5); //  speed for motor A
  analogWrite(ENB, 52); //   speed for motor B

  digitalWrite(LR, HIGH);
  digitalWrite(LC, HIGH);      // All Green light turn on
  digitalWrite(LL, HIGH);

  analogWrite(LRR, 0);
  analogWrite(LCR, 0);       // All Red light turn off
  analogWrite(LLR, 0);
  
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
}

void moveBackward() {
  digitalWrite(LR, LOW);
  digitalWrite(LC, LOW);        // All Green light turn off
  digitalWrite(LL, LOW);

  analogWrite(LRR, 255);
  analogWrite(LCR, 255);       // All Red light turn on
  analogWrite(LLR, 255);
  
 
  digitalWrite(LEFT_MOTOR_PIN1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}

void turnRight() {
  analogWrite(ENA, 100); //  speed for motor A
  analogWrite(ENB, 90); //  speed for motor B
  
  digitalWrite(LR, HIGH);
  digitalWrite(LC, LOW);            // right side Green light turn on
  digitalWrite(LL, LOW);

  analogWrite(LRR, 0);
  analogWrite(LCR, 255);         // Left and Middel Red light turn on
  analogWrite(LLR, 255);
  
  
  digitalWrite(LEFT_MOTOR_PIN1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
}

void turnLeft() {
  analogWrite(ENA, 100); //  speed for motor A
  analogWrite(ENB, 90); //  speed for motor B
  
  digitalWrite(LR, LOW);
  digitalWrite(LC, LOW);         // left side Green light turn on
  digitalWrite(LL, HIGH);

  analogWrite(LRR, 255);
  analogWrite(LCR, 255);         // right and Middel Red light turn on
  analogWrite(LLR, 0);
  
 
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}

void Stop() {
  digitalWrite(LR, LOW);
  digitalWrite(LC, LOW);
  digitalWrite(LL, LOW);

  analogWrite(LRR, 255);
  analogWrite(LCR, 255);
  analogWrite(LLR, 255);
  
  analogWrite(ENA, 70.5); //  speed for motor A
  analogWrite(ENB, 60); //  speed for motor B
 
  digitalWrite(LEFT_MOTOR_PIN1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  delay(50);
  
  analogWrite(ENA, 0); //  speed for motor A
  analogWrite(ENB, 0); //  speed for motor B
  
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}
