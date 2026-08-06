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
const float aceleracao = 3000.0;        // rampa suave ao ganhar velocidade (evita ruido/engasgo)
const float aceleracaoParada = 3000.0; // rampa rapida ao parar/inverter sentido (evita lag ao trocar de comando)

// sentido atual de cada roda (-1, 0 ou +1): usado pra saber se um comando novo
// esta invertendo o sentido de giro, caso em que precisa desacelerar rapido
int sinalFL = 0, sinalFR = 0, sinalRL = 0, sinalRR = 0;

// driver TB6600: SW1=OFF, SW2=ON, SW3=OFF -> 1/8 microstep, motor 1.8 grau (200 steps/volta)
const float RAIO_RODA = 0.045; // metros
const long PULSOS_POR_REV = 2000;
const float STEPS_POR_METRO = PULSOS_POR_REV / (2.0 * PI * RAIO_RODA);

// timeout de seguranca no proprio firmware: se a serial degradar (ex: ruido,
// desconexao) e nenhum comando valido novo chegar, para sozinho em vez de
// continuar executando o ultimo move() para sempre
const unsigned long SERIAL_TIMEOUT_MS = 500;
unsigned long ultimoComandoValido = 0;
bool roboParado = true;

char bufferSerial[32];
uint8_t posBufferSerial = 0;

// ultimo comando efetivamente aplicado aos motores: usado pra nao reemitir
// setMaxSpeed()/move() quando o /cmd_vel novo repete o mesmo valor
float ultimoVx = 0.0;
float ultimoVy = 0.0;
float ultimoW = 0.0;
const float EPSILON_VEL = 0.001;

bool comandoMudou(float Vx, float Vy, float W) {
  return abs(Vx - ultimoVx) > EPSILON_VEL ||
         abs(Vy - ultimoVy) > EPSILON_VEL ||
         abs(W - ultimoW) > EPSILON_VEL;
}

void setup() {
  Serial.begin(115200);
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

void aplicarMovimento(AccelStepper &motor, int &sinalAtual, float prop) {
  int sinalNovo = (prop >= 0) ? 1 : -1;

  // se ja estava girando e o sentido inverteu, desacelera rapido em vez de
  // usar a mesma rampa suave da partida - senao o tempo de resposta fica
  // proporcional a velocidade atual (quanto mais tempo segurou o comando
  // anterior, mais demora pra atender o proximo)
  bool invertendo = (sinalAtual != 0) && (sinalNovo != sinalAtual);
  motor.setAcceleration(invertendo ? aceleracaoParada : aceleracao);

  float vel = constrain(abs(prop) * STEPS_POR_METRO, 0.0, velocidadeMax);
  motor.setMaxSpeed(vel);

  // so redefine o alvo (move()) quando o sentido muda ou o motor estava parado:
  // no mesmo sentido, so mudar setMaxSpeed() ja e suficiente pro AccelStepper
  // reajustar a rampa - chamar move() de novo a toa recalcula o perfil de
  // aceleracao (computeNewSpeed()) sem necessidade a cada pacote
  if (sinalNovo != sinalAtual) {
    motor.move(1000000L * sinalNovo);
  }
  sinalAtual = sinalNovo;
}

void moverRoboContinuo(float Vx, float Vy, float W) {
  // propX ja e a velocidade de superficie de cada roda em m/s (cinematica
  // mecanum). Antes isso so era normalizado pela maior roda, fazendo a
  // dominante sempre bater em velocidadeMax (cheio) e ignorando a magnitude
  // real pedida pelo /cmd_vel - por isso o robo sempre "corria" e fazia
  // barulho, mesmo em velocidades baixas.
  float propFL = Vx + Vy + (W * L);
  float propFR = Vx - Vy - (W * L);
  float propRL = Vx - Vy + (W * L);
  float propRR = Vx + Vy - (W * L);

  aplicarMovimento(motorFL, sinalFL, propFL);
  aplicarMovimento(motorFR, sinalFR, propFR);
  aplicarMovimento(motorRL, sinalRL, propRL);
  aplicarMovimento(motorRR, sinalRR, propRR);
}

void pararRobo() {
  // parar tambem usa a rampa rapida: sempre queremos que a parada seja
  // pronta, nao a rampa suave de partida
  motorFL.setAcceleration(aceleracaoParada);
  motorFR.setAcceleration(aceleracaoParada);
  motorRL.setAcceleration(aceleracaoParada);
  motorRR.setAcceleration(aceleracaoParada);

  motorFL.stop();
  motorFR.stop();
  motorRL.stop();
  motorRR.stop();

  sinalFL = sinalFR = sinalRL = sinalRR = 0;
}

void loop() {
  // le so os bytes que ja estao no buffer da UART, sem esperar o resto da
  // linha chegar: Serial.readBytesUntil() bloqueava ate ~100ms quando so
  // parte da linha tinha chegado, o que com o /cmd_vel do ROS publicando
  // continuamente (varias linhas por segundo, quase sempre fragmentadas
  // entre leituras) travava o loop() e os motorX.run() por perto disso a
  // cada pacote - daí o engasgo. Via Monitor Serial manual isso nao aparecia
  // porque uma linha digitada chega inteira de uma vez, com tempo ocioso
  // de sobra entre comandos.
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      bufferSerial[posBufferSerial] = '\0';
      posBufferSerial = 0;

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

        // so reemite setMaxSpeed()/move() se o comando realmente mudou: chamar
        // isso de novo a cada pacote (varias vezes por segundo) mesmo com a
        // mesma velocidade reinicia o perfil de aceleracao do AccelStepper e
        // causa os engasgos/ruido durante a partida
        if (comandoMudou(Vx, Vy, W)) {
          if (Vx == 0.0 && Vy == 0.0 && W == 0.0) {
            pararRobo();
            roboParado = true;
          } else {
            moverRoboContinuo(Vx, Vy, W);
            roboParado = false;
          }
          ultimoVx = Vx;
          ultimoVy = Vy;
          ultimoW = W;
        }
      }
    } else if (posBufferSerial < sizeof(bufferSerial) - 1) {
      bufferSerial[posBufferSerial++] = c;
    }
    // linha maior que o buffer: os bytes excedentes sao descartados ate o
    // proximo '\n' em vez de estourar o array
  }

  // parada de seguranca: se a serial parar de entregar comandos validos
  // (desconexao, ruido, etc.), nao fica executando o ultimo move() pra sempre
  if (!roboParado && (millis() - ultimoComandoValido > SERIAL_TIMEOUT_MS)) {
    pararRobo();
    roboParado = true;
    ultimoVx = ultimoVy = ultimoW = 0.0;
  }

  motorFR.run();
  motorFL.run();
  motorRR.run();
  motorRL.run();
}