// Kaua Silva Oliveira - FBOT@WORK
// PROJETO: ROBÔ MECANUM 4WD + TELEMETRIA DUPLA MPU
// VERSÃO: ULTRA-FLUIDA (Foco em Baixa Latência)

#include <Wire.h>
#include <AccelStepper.h>

// ==========================================
// CONFIGURAÇÕES DE HARDWARE
// ==========================================
#define WHEEL_DIAMETER 0.10      
#define LR_DISTANCE 0.30         
#define FR_DISTANCE 0.30         
#define STEPS_PER_REV 1300       //Ajuste conforme os switches do TB6600
#define MAX_STEPS_S 4000.0       

const float GEOMETRIC_CONSTANT = (LR_DISTANCE / 2.0) + (FR_DISTANCE / 2.0);
const float M_TO_STEPS = 1.0 / ((PI * WHEEL_DIAMETER) / (float)STEPS_PER_REV);

// ==========================================
// PINOS DOS MOTORES
// ==========================================
AccelStepper motorFL(1, 31, 30); 
AccelStepper motorFR(1, 33, 32); 
AccelStepper motorRL(1, 35, 34); 
AccelStepper motorRR(1, 37, 36); 

// ==========================================
// VARIÁVEIS DOS SENSORES (MPU6050)
// ==========================================
#define MPU1 0x68
#define MPU2 0x69

float accX1, accY1, accZ1, gyroX1, gyroY1, gyroZ1, temp1;
float accX2, accY2, accZ2, gyroX2, gyroY2, gyroZ2, temp2;

// ==========================================
// CONTROLE E TIMING
// ==========================================
float req_vx = 0.0, req_vy = 0.0, req_vw = 0.0;
unsigned long lastCommandTime = 0;
unsigned long lastSensorTime = 0;
const unsigned long SENSOR_INTERVAL = 500; // 0.5s para total fluidez dos motores

// ==========================================
// SETUP
// ==========================================
void setup() {
  // Aumentado para 115200 para liberar o processador mais rápido
  Serial.begin(115200); 
  Wire.begin();
  Wire.setClock(400000); 

  iniciarMPU(MPU1);
  iniciarMPU(MPU2);

  motorFL.setMaxSpeed(MAX_STEPS_S);
  motorFR.setMaxSpeed(MAX_STEPS_S);
  motorRL.setMaxSpeed(MAX_STEPS_S);
  motorRR.setMaxSpeed(MAX_STEPS_S);

  // Garante que o robô comece parado
  pararMotores();
}

// ==========================================
// LOOP PRINCIPAL
// ==========================================
void loop() {
  // 1. RECEBER COMANDOS (Lógica de busca por marcadores < >)
  receberComandos();

  // 2. FAILSAFE (Se não receber comandos por 1s, para)
  if (millis() - lastCommandTime > 1000) {
    req_vx = 0; req_vy = 0; req_vw = 0;
  }

  // 3. ATUALIZAR VELOCIDADES
  processarCinematica();

  // 4. SENSORES E TELEMETRIA (A cada 500ms)
  if (millis() - lastSensorTime >= SENSOR_INTERVAL) {
    atualizarSensores();
    enviarTelemetria();
    lastSensorTime = millis();
  }

  // 5. EXECUTAR PASSOS (Prioridade máxima)
  motorFL.runSpeed();
  motorFR.runSpeed();
  motorRL.runSpeed();
  motorRR.runSpeed();
}

// ==========================================
// FUNÇÕES DE MOVIMENTO
// ==========================================
void processarCinematica() {
  // Cálculo de velocidades para cada roda (Mecanun)
  float vFL = (req_vx + req_vy - (req_vw * GEOMETRIC_CONSTANT)) * M_TO_STEPS;
  float vFR = (req_vx - req_vy + (req_vw * GEOMETRIC_CONSTANT)) * M_TO_STEPS;
  float vRL = (req_vx - req_vy - (req_vw * GEOMETRIC_CONSTANT)) * M_TO_STEPS;
  float vRR = (req_vx + req_vy + (req_vw * GEOMETRIC_CONSTANT)) * M_TO_STEPS;

  motorFL.setSpeed(vFL);
  motorFR.setSpeed(vFR);
  motorRL.setSpeed(vRL);
  motorRR.setSpeed(vRR);
}

void pararMotores() {
  motorFL.setSpeed(0);
  motorFR.setSpeed(0);
  motorRL.setSpeed(0);
  motorRR.setSpeed(0);
}

// ==========================================
// COMUNICAÇÃO SERIAL (INPUT)
// ==========================================
void receberComandos() {
  while (Serial.available() > 0) {
    char rc = Serial.read();
    if (rc == '<') {
      String data = Serial.readStringUntil('>');
      int firstComma = data.indexOf(',');
      int secondComma = data.lastIndexOf(',');
      
      if (firstComma != -1 && secondComma != -1) {
        req_vx = data.substring(0, firstComma).toFloat();
        req_vy = data.substring(firstComma + 1, secondComma).toFloat();
        req_vw = data.substring(secondComma + 1).toFloat();
        lastCommandTime = millis();
      }
    }
  }
}

// ==========================================
// TELEMETRIA (OUTPUT)
// ==========================================
void atualizarSensores() {
  lerMPU(MPU1, accX1, accY1, accZ1, gyroX1, gyroY1, gyroZ1, temp1);
  lerMPU(MPU2, accX2, accY2, accZ2, gyroX2, gyroY2, gyroZ2, temp2);
}

void enviarTelemetria() {
  // Cálculos de Roll e Pitch feitos apenas no envio para poupar CPU
  float roll1  = atan2(accY1, accZ1) * 57.3;
  float pitch1 = atan2(-accX1, sqrt(accY1*accY1 + accZ1*accZ1)) * 57.3;
  float roll2  = atan2(accY2, accZ2) * 57.3;
  float pitch2 = atan2(-accX2, sqrt(accY2*accY2 + accZ2*accZ2)) * 57.3;

  Serial.print(roll1);  Serial.print(","); Serial.print(pitch1); Serial.print(",");
  Serial.print(accX1);  Serial.print(","); Serial.print(accY1);  Serial.print(",");
  Serial.print(accZ1);  Serial.print(","); Serial.print(gyroX1); Serial.print(",");
  Serial.print(gyroY1); Serial.print(","); Serial.print(gyroZ1); Serial.print(",");
  Serial.print(temp1);  Serial.print(",");
  
  Serial.print(roll2);  Serial.print(","); Serial.print(pitch2); Serial.print(",");
  Serial.print(accX2);  Serial.print(","); Serial.print(accY2);  Serial.print(",");
  Serial.print(accZ2);  Serial.print(","); Serial.print(gyroX2); Serial.print(",");
  Serial.print(gyroY2); Serial.print(","); Serial.print(gyroZ2); Serial.print(",");
  Serial.print(temp2);  Serial.print(",");
  
  Serial.print(0); Serial.print(","); // Botão simulado
  Serial.print((abs(req_vx) > 0.05) ? 1 : 0); // Status motor
  Serial.println(";");
}

// ==========================================
// FUNÇÕES I2C / MPU6050
// ==========================================
void iniciarMPU(int endereco) {
  Wire.beginTransmission(endereco);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void lerMPU(int endereco, float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &temp) {
  Wire.beginTransmission(endereco);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(endereco, 14, true);
  
  if (Wire.available() >= 14) {
    int16_t rawAX = Wire.read() << 8 | Wire.read();
    int16_t rawAY = Wire.read() << 8 | Wire.read();
    int16_t rawAZ = Wire.read() << 8 | Wire.read();
    int16_t rawT  = Wire.read() << 8 | Wire.read();
    int16_t rawGX = Wire.read() << 8 | Wire.read();
    int16_t rawGY = Wire.read() << 8 | Wire.read();
    int16_t rawGZ = Wire.read() << 8 | Wire.read();

    ax = rawAX / 16384.0;
    ay = rawAY / 16384.0;
    az = rawAZ / 16384.0;
    gx = rawGX / 131.0;
    gy = rawGY / 131.0;
    gz = rawGZ / 131.0;
    temp = (rawT / 340.0) + 36.53;
  }
}