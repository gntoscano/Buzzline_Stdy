#define NUM_SENSORS 5

const int sensorPins[NUM_SENSORS] = {34, 35, 32, 33, 25}; // ajuste conforme o seu ESP32

int weights[NUM_SENSORS] = {-2, -1, 0, 1, 2};  // posição relativa dos sensores

int sensorValues[NUM_SENSORS];  // valores lidos dos sensores
int invertedValues[NUM_SENSORS];  // valores invertidos para peso

const int blackMin = 200;  // valor típico da linha preta
const int whiteMax = 3000; // valor típico do branco

// Lê e normaliza os valores dos sensores
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);

    // Inverte e normaliza os valores para que preto = alto valor
    int val = constrain(sensorValues[i], blackMin, whiteMax);
    invertedValues[i] = map(whiteMax - val, 0, whiteMax - blackMin, 0, 1000);
  }
}

// Calcula a posição da linha com base na média ponderada
float calculateLinePosition() {
  long weightedSum = 0;
  long total = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    weightedSum += (long)invertedValues[i] * weights[i];
    total += invertedValues[i];
  }

  if (total == 0) {
    return 0;  // Nenhum sensor detectou linha, retorna centro
  }

  return (float)weightedSum / total;
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }
}

void loop() {
  readSensors();
  float position = calculateLinePosition();

  Serial.print("Posição da linha: ");
  Serial.println(position);  // de -2 (esquerda) até +2 (direita)

  delay(100);
}
