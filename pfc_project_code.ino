// Define pins for potentiometer and motor encoder
const int ldr_01 = A2;       // LDR module 1
const int ldr_02 = A3;       // LDR module 2
const int motorEncoderA = 2; // Motor encoder pulse A
const int motorEncoderB = 3; // Motor encoder pulse B

// Define motor control pins
const int EnableA = 6;      // Motor PWM control
const int motorIn1 = 4; // Motor direction control
const int motorIn2 = 5;

// Define variables for PID control
double Kp = 2.5;  // Proportional gain //best until now: 2.5
double Ki = 0.5;  // Integral gain
double Kd = 0.05; // Derivative gain //best until now: 0.05

double prevError = 0;
long previousTime = 0;
double integral = 0;

// Interrupt service routine to update motor encoder count
//volatile int encoderCount = 0;        // Motor position in encoder counts
//int prevEncoderCount = 0;             // Previous motor position
//int centralReferenceEncoderCount = 0; // Central reference position
int rotationDirection = 1;            // 1 for clockwise, -1 for anticlockwise

const int encoderCountsPerRevolution = 360;  // Adjust based on your encoder specification

// Define variables for gain control
float gain = 1.0; // Initial gain value

void setup() {
  Serial.begin(9600);
  
  // Setup potentiometer and motor encoder pins
  pinMode(ldr_01, INPUT);
  pinMode(ldr_02, INPUT);
  pinMode(motorEncoderA, INPUT);
  pinMode(motorEncoderB, INPUT);

  // Setup motor control pins
  pinMode(EnableA, OUTPUT);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);

  // Attach interrupt for motor encoder
  attachInterrupt(digitalPinToInterrupt(motorEncoderA), updateEncoder, CHANGE);
  // Wait for the motor to stabilize and then set the initial position as the central reference
  delay(1000);  // Adjust the delay as needed
  centralReferenceEncoderCount = encoderCount;
}


void loop() {
  // put your main code here, to run repeatedly:
  // Calculate the absolute motor position
  int absoluteEncoderCount = encoderCount;

  // Calculate the relative position to the central reference
  int relativePosition = absoluteEncoderCount - centralReferenceEncoderCount;

  // Adjust relative position to be in the range [0, 360)
  relativePosition = (relativePosition + encoderCountsPerRevolution) % encoderCountsPerRevolution;

  // Convert relative encoder counts to degrees
  float degrees = map(relativePosition * rotationDirection, 0, encoderCountsPerRevolution, 0, 360);

  if ( (degrees>=0) && (degrees<=180))
  {
      degrees = degrees;
  }
  else if ( (degrees>=180) && (degrees<=360))
  {
      degrees = map(degrees, 180, 360, -180, 0);
  }
  // Print the relative motor shaft position in degrees
  Serial.print("\n");
  Serial.print("Motor Shaft Position: ");
  Serial.print(degrees);
  Serial.println(" degrees");

  // for the Derivative in PD Controller
  long currentTime = millis();
  float deltaTime = ((float)(currentTime - previousTime)) / 1000.0;

  // Read angle from potentiometer
  double setpoint = getAngle();

  // Read angle from motor encoder
  //double angle = degrees;

  // Calculate error
  double error = setpoint - degrees;
  double deltaError = (error - prevError) / deltaTime;

  // Calculate PID terms
  double proportional = Kp * error;
  //integral += Ki * error;
  double derivative = Kd * deltaError;

  // Calculate PID output
  double output = proportional + derivative; //+ integral;
  output = constrain(output, -255, 255); // Limit output to motor PWM range

  // Apply the output to the motor
  if (output > 0) {
    digitalWrite(motorIn1, LOW); // Set motor direction
    digitalWrite(motorIn2, HIGH);
  } else {
    digitalWrite(motorIn1, HIGH); // Set motor direction
    digitalWrite(motorIn2, LOW);
  }
  analogWrite(EnableA, abs(output)); // Set motor speed // see for improvement here

  // Update previous error for the next iteration
  prevError = error;
  previousTime = currentTime;

  Serial.print("\n");
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print("\n");
  Serial.print("Control signal: ");
  Serial.print(output);
  
  analogWrite(EnableA, 0);
  delay(100); // Adjust delay as needed
}


// Function to get the angle from the potentiometer voltage
double getAngle() {
  int rawValue = analogRead(potPin);
  //double voltage = rawValue * (5.0 / 1023.0); // Convert to voltage
  //double angle = map(voltage, 0, 5, -135, 135); // Map to angle range
  // map the raw value to an angle range (-135 to 135 degrees)
  float angle = map(rawValue, 0, 1023, -135, 135);
  
  // print the angle to the serial monitor
  Serial.print("Potentiometer Angle: ");
  Serial.print(angle);
  Serial.println(" degrees");
  
  return angle * gain; // Apply gain
}


void updateEncoder() {
  // Read encoder signals
  int A = digitalRead(motorEncoderA);
  int B = digitalRead(motorEncoderB);

  // Implement quadrature decoding logic
  if (A != B) {
    encoderCount += rotationDirection;
  } else {
    encoderCount -= rotationDirection;
  }
}
