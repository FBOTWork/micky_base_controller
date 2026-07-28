#include <AccelStepper.h>

// Definição dos pinos: [Pulse, Dir]
#define PUL_FR 39
#define DIR_FR 37
#define PUL_FL 47
#define DIR_FL 45
#define PUL_RR 43
#define DIR_RR 41
#define PUL_RL 51
#define DIR_RL 49

// Mapeamento pino->objeto CORRIGIDO (confirmado via teste_motores_individual_v2.ino):
// o canal fisicamente ligado ao motor FL real usa os pinos definidos como PUL_RR/DIR_RR,
// e o canal ligado ao motor RR real usa os pinos definidos como PUL_FL/DIR_FL.
AccelStepper motorFR(1, PUL_FR, DIR_FR);
AccelStepper motorRR(1, PUL_FL, DIR_FL);
AccelStepper motorFL(1, PUL_RR, DIR_RR);
AccelStepper motorRL(1, PUL_RL, DIR_RL);

const float L = 0.38;
const float velocidadeMax = 4000.0;
const float aceleracao = 700.0;

// timeout de seguranca no proprio firmware: se a serial degradar (ex: ruido,
// desconexao) e nenhum comando valido novo chegar, para sozinho em vez de
// continuar executando o ultimo move() para sempre
const unsigned long SERIAL_TIMEOUT_MS = 200;
unsigned long ultimoComandoValido = 0;
bool roboParado = true;

char bufferSerial[32];

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  Serial.println("Base Mecanum Pronta - Aguardando comandos do Python...");

  // Polaridade final: padrão simétrico esquerda/direita (confirmado por teste individual)
  motorFL.setPinsInverted(true, false, false);
  motorRL.setPinsInverted(true, false, false);
  motorRR.setPinsInverted(false, false, false);
  // motorFR permanece sem inversão (default)

  AccelStepper* motores[] = {&motorFR, &motorFL, &motorRR, &motorRL};
  for (int i = 0; i < 4; i++) {
    motores[i]->setMinPulseWidth(30);
    motores[i]->setMaxSpeed(velocidadeMax);
    motores[i]->setAcceleration(aceleracao);
  }
}

void moverRoboContinuo(float Vx, float Vy, float W) {
  float propFL = Vx + Vy + (W * L);
  float propFR = Vx - Vy - (W * L);
  float propRL = Vx - Vy + (W * L);
  float propRR = Vx + Vy - (W * L);

  float maxProp = max(max(abs(propFL), abs(propFR)), max(abs(propRL), abs(propRR)));
  if (maxProp == 0) maxProp = 1.0;

  motorFL.setMaxSpeed(velocidadeMax * (abs(propFL) / maxProp));
  motorFR.setMaxSpeed(velocidadeMax * (abs(propFR) / maxProp));
  motorRL.setMaxSpeed(velocidadeMax * (abs(propRL) / maxProp));
  motorRR.setMaxSpeed(velocidadeMax * (abs(propRR) / maxProp));

  long infinito = 1000000;
  motorFL.move(infinito * (propFL >= 0 ? 1 : -1));
  motorFR.move(infinito * (propFR >= 0 ? 1 : -1));
  motorRL.move(infinito * (propRL >= 0 ? 1 : -1));
  motorRR.move(infinito * (propRR >= 0 ? 1 : -1));
}

void pararRobo() {
  motorFL.stop();
  motorFR.stop();
  motorRL.stop();
  motorRR.stop();
}

void loop() {
  if (Serial.available() > 0) {
    // buffer fixo em vez de String: evita fragmentacao de heap no loop, que
    // com uptime longo degradava o parsing e travava o robo na ultima
    // velocidade recebida (String aloca/realoca memoria dinamicamente)
    size_t len = Serial.readBytesUntil('\n', bufferSerial, sizeof(bufferSerial) - 1);
    bufferSerial[len] = '\0';

    // strtok+atof em vez de sscanf("%f"): a avr-libc padrao do Arduino Mega/Uno
    // nao suporta ponto flutuante no scanf (falharia silenciosamente)
    char* tokVx = strtok(bufferSerial, ",");
    char* tokVy = tokVx ? strtok(NULL, ",") : NULL;
    char* tokW  = tokVy ? strtok(NULL, ",") : NULL;

    if (tokVx && tokVy && tokW) {
      float Vx = atof(tokVx);
      float Vy = atof(tokVy);
      float W  = atof(tokW);
      ultimoComandoValido = millis();

      if (Vx == 0.0 && Vy == 0.0 && W == 0.0) {
        pararRobo();
        roboParado = true;
      } else {
        moverRoboContinuo(Vx, Vy, W);
        roboParado = false;
      }
    }
  }

  // parada de seguranca: se a serial parar de entregar comandos validos
  // (desconexao, ruido, etc.), nao fica executando o ultimo move() pra sempre
  if (!roboParado && (millis() - ultimoComandoValido > SERIAL_TIMEOUT_MS)) {
    pararRobo();
    roboParado = true;
  }

  motorFR.run();
  motorFL.run();
  motorRR.run();
  motorRL.run();
}