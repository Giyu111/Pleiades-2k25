
// Sensor pins
int sensors[] = {22, 23, 24, 25, 26, 27, 28, 29}; 
int sensorValues[8];

// Motor pins
#define ENA 5     // PWM pin for Left Motor
#define ENB 6     // PWM pin for Right Motor
#define IN1 7     // Direction pin for Left Motor
#define IN2 8     // Direction pin for Left Motor
#define IN3 9     // Direction pin for Right Motor
#define IN4 10    // Direction pin for Right Motor

// PID constants (will be tuned later)
float Kp = 30;    
float Ki = 0;
float Kd = 15;

float lastError = 0;
float integral = 0;

// Base speed
int baseSpeed = 120;

void setup()
{
  for (int i = 0; i < 8; i++) 
  {
    pinMode(sensors[i], INPUT);
  }
  
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);
}

void loop()
{
  int position = readSensors();
  float error = position - 3500;
  integral += error;
  float derivative = error - lastError;
  float output = Kp * error + Ki * integral + Kd * derivative;

  int leftSpeed = baseSpeed + output;
  int rightSpeed = baseSpeed - output;

  bool isIntersection = isPlusIntersection();
  bool isCurve = isSharpCurve();

  if (isIntersection) 
  {
    leftSpeed = 180;
    rightSpeed = 180;
  } else if (isCurve)
  {
    // Slingshot turn mode
    leftSpeed = baseSpeed + (output * 1.5);   // Boost turning more
    rightSpeed = baseSpeed - (output * 1.5);
  } 
  else 
  {
    // Normal
    leftSpeed = constrain(leftSpeed, 60, 200);
    rightSpeed = constrain(rightSpeed, 60, 200);
  }

  setMotor(leftSpeed, rightSpeed);
  lastError = error;
}


int readSensors() 
{
  long weightedSum = 0;
  int sum = 0;
  
  for (int i = 0; i < 8; i++) 
  {
    sensorValues[i] = digitalRead(sensors[i]);
    if (sensorValues[i] == 1) 
    {
      weightedSum += i * 1000;
      sum++;
    }
  }
  
  if (sum == 0) 
  {
    // No line detected
    if (lastError > 0) return 7000; 
    else return 0;
  }
  
  return weightedSum / sum;
}

void setMotor(int leftSpeed, int rightSpeed) 
{
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);

  if (leftSpeed >= 0) 
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else 
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  if (rightSpeed >= 0) 
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } 
  else 
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

bool isSharpCurve() {
  int activeSensors = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] == 1) activeSensors++;
  }
  
  if (activeSensors <= 2) {
    // Sharp curve detected
    return true;
  }
  return false;
}
