// Define pins for LDRs and motor
const int ldr_01 = A2;       // LDR module 1
const int ldr_02 = A3;       // LDR module 2
const int motorIn1 = 4;      // Motor direction control
const int motorIn2 = 5;      // Motor direction control
const int EnableA = 6;       // Motor PWM control

// Defining variables for P controller
double Kp = 0.5;             // Proportional gain
const int differenceThreshold = 100;  // Threshold for DC motor activation based on LDR difference

void setup() 
{
  Serial.begin(9600);
  pinMode(ldr_01, INPUT);
  pinMode(ldr_02, INPUT);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(EnableA, OUTPUT);
}
void loop() 
{
  int ldrValue_01 = analogRead(ldr_01);
  int ldrValue_02 = analogRead(ldr_02);

  int difference = ldrValue_01 - ldrValue_02;

  // Implement P controller
  double output = Kp * difference;

  // Ensure the motor doesn't rotate if difference is below threshold
  if (abs(difference) < differenceThreshold) 
  {
    output = 0;
  }
  // Determine motor direction based on which LDR has the lower value
  if (ldrValue_01 < ldrValue_02) 
  {
    digitalWrite(motorIn1, LOW);  // Clockwise direction
    digitalWrite(motorIn2, HIGH);
  } 
  else 
  {
    digitalWrite(motorIn1, HIGH); // Anticlockwise direction
    digitalWrite(motorIn2, LOW);
  }
  // Apply PWM to the motor based on P controller output
  if (output > 0) 
  {
    analogWrite(EnableA, output); // Set motor speed
  } 
  else 
  {
    analogWrite(EnableA, -output); // Set motor speed
  }

  // Printing values 
  Serial.print("\nLDR_01: ");
  Serial.print(ldrValue_01);
  Serial.print("\n, LDR_02: ");
  Serial.print(ldrValue_02);
  Serial.print("\n, Difference: ");
  Serial.print(difference);
  Serial.print("\n, Output: ");
  Serial.println(output);

  delay(100); 
}
