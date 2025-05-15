#define NUM_SENSORS 5

const int sensorPins[NUM_SENSORS] = {34, 35, 32, 33, 25};
int weights[NUM_SENSORS] = {-2, -1, 0, 1, 2};

int sensorValues[NUM_SENSORS];
int invertedValues[NUM_SENSORS];
const int blackMin = 200;
const int whiteMax = 3000;

// PID
float Kp = 30.0;
float Ki = 0.0;
float Kd = 15.0;

float error = 0;
float lastError = 0;
float integral = 0;

// Motores
const int IN1_A = 14;
const int IN2_A = 12;
const int PWM_A = 26;

const int IN1_B = 13;
const int IN2_B = 15;
const int PWM_B = 27;

const int baseSpeed = 180;  // valor base PWM (0-255)

void setup()
{
  Serial.begin(115200);
  
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  pinMode(IN1_B, OUTPUT);
  pinMode(IN2_B, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
}

void readSensors()
{
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    sensorValues[i] = analogRead(sensorPins[i]);
    int val = constrain(sensorValues[i], blackMin, whiteMax);
    invertedValues[i] = map(whiteMax - val, 0, whiteMax - blackMin, 0, 1000);
  }
}

float calculateLinePosition()
{
  long weightedSum = 0;
  long total = 0;

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    weightedSum += (long)invertedValues[i] * weights[i];
    total += invertedValues[i];
  }

  if (total == 0) return 0;

  return (float)weightedSum / total;
}

void controlMotors(float pid)
{
  int leftSpeed = baseSpeed + pid;
  int rightSpeed = baseSpeed - pid;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Motor Esquerdo
  digitalWrite(IN1_A, HIGH);
  digitalWrite(IN2_A, LOW);
  ledcWrite(0, leftSpeed);  // Canal 0

  // Motor Direito
  digitalWrite(IN1_B, HIGH);
  digitalWrite(IN2_B, LOW);
  ledcWrite(1, rightSpeed); // Canal 1
}

void loop()
{
  readSensors();
  float position = calculateLinePosition();

  error = 0 - position;
  integral += error;
  float derivative = error - lastError;

  float pid = Kp * error + Ki * integral + Kd * derivative;

  controlMotors(pid);

  lastError = error;

  Serial.print("Posição: ");
  Serial.print(position);
  Serial.print(" | PID: ");
  Serial.println(pid);

  delay(20);
}

void ledcSetupAll()
{
  ledcSetup(0, 5000, 8); // canal 0, 5kHz, 8 bits
  ledcSetup(1, 5000, 8); // canal 1
  ledcAttachPin(PWM_A, 0);
  ledcAttachPin(PWM_B, 1);
}
