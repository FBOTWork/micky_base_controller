/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F103C8T6 - Controle 4 Motores + 2x MPU6050 (I2C1 e I2C2)
  *                   - IMU Dianteira: PB8 (SCL), PB9 (SDA) -> I2C1 Remap
  *                   - IMU Traseira:  PB10 (SCL), PB11 (SDA) -> I2C2
  *                   Revisado: Roll/Pitch reais, clamp de Yaw, otimização de CPU
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// Registradores da MPU6050
#define MPU6050_ADDR         0xD0 // Endereço padrão (AD0 = GND)
#define SMPLRT_DIV_REG       0x19
#define CONFIG_REG           0x1A
#define GYRO_CONFIG_REG      0x1B
#define ACCEL_CONFIG_REG     0x1C
#define PWR_MGMT_1_REG       0x6B
#define ACCEL_XOUT_H_REG     0x3B
#define WHO_AM_I_REG         0x75

// Estrutura para gerenciar múltiplas IMUs
typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t i2c_addr;
    float ax, ay, az;
    float gx, gy, gz;
    float gyro_z_bias;
    float roll, pitch, yaw;
    uint8_t whoami;
    uint32_t falhas;
    bool presente;              // detectada no boot; se false, é ignorada até reiniciar
    uint32_t ultima_leitura;
    uint32_t proxima_leitura;
    const char* prefix;
} MPU_State;

// Instanciação das IMUs em barramentos independentes
MPU_State imuFront = {.hi2c = &hi2c1, .i2c_addr = MPU6050_ADDR, .prefix = "IMU_F"};
MPU_State imuRear  = {.hi2c = &hi2c2, .i2c_addr = MPU6050_ADDR, .prefix = "IMU_R"};

uint32_t imu_recuperacoes = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN 0 */
#define PULSOS_POR_REV 1600.0f
#define VELOCIDADE_MAX 4000.0f
#define ACELERACAO 600.0f
#define ACELERACAO_PARADA 2000.0f
#define RAIO_RODA 0.05f
#define PI 3.14159265359f
#define STEPS_POR_METRO (PULSOS_POR_REV / (2.0f * PI * RAIO_RODA))
#define FREQ_TIMER 50000.0f
#define FASE_POR_HZ (4294967296.0f / FREQ_TIMER)

typedef struct {
    GPIO_TypeDef* port_pul; uint16_t pin_pul;
    GPIO_TypeDef* port_dir; uint16_t pin_dir;
    float target_speed, current_speed, acceleration;
    uint32_t step_phase; volatile uint32_t step_increment;
    int direction; bool invert_dir;
} StepperMotor;

StepperMotor motorFL, motorFR, motorRL, motorRR;

#define SERIAL_TIMEOUT_MS 500
uint32_t ultimoComandoValido = 0;
bool roboParado = true;
float ultimoVFL = 0.0f, ultimoVFR = 0.0f, ultimoVRL = 0.0f, ultimoVRR = 0.0f;
#define EPSILON_VEL 0.001f

#define RX_BUFFER_SIZE 128
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_index = 0;
volatile bool flag_comando_recebido = false;

// Interrupção: Recebe comandos do ROS
void Receber_Dados_USB(uint8_t* Buf, uint32_t Len) {
    for(uint32_t i = 0; i < Len; i++) {
        if (Buf[i] == '\n') {
            rx_buffer[rx_index] = '\0';
            flag_comando_recebido = true;
            rx_index = 0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        } else {
            if (rx_index < RX_BUFFER_SIZE - 1) rx_buffer[rx_index++] = Buf[i];
        }
    }
}

// Inicialização dos Motores
void Motores_Init(void) {
    motorFL.port_pul = GPIOB; motorFL.pin_pul = GPIO_PIN_15;
    motorFL.port_dir = GPIOB; motorFL.pin_dir = GPIO_PIN_14; motorFL.invert_dir = true;

    motorFR.port_pul = GPIOB; motorFR.pin_pul = GPIO_PIN_6;
    motorFR.port_dir = GPIOB; motorFR.pin_dir = GPIO_PIN_5; motorFR.invert_dir = false;

    motorRL.port_pul = GPIOB; motorRL.pin_pul = GPIO_PIN_13;
    motorRL.port_dir = GPIOB; motorRL.pin_dir = GPIO_PIN_12; motorRL.invert_dir = true;

    motorRR.port_pul = GPIOB; motorRR.pin_pul = GPIO_PIN_3;
    motorRR.port_dir = GPIOB; motorRR.pin_dir = GPIO_PIN_4; motorRR.invert_dir = false;
}

void Aplicar_Movimento(StepperMotor* motor, float velocidade_linear) {
    float vel_passos = velocidade_linear * STEPS_POR_METRO;
    if (vel_passos > VELOCIDADE_MAX) vel_passos = VELOCIDADE_MAX;
    if (vel_passos < -VELOCIDADE_MAX) vel_passos = -VELOCIDADE_MAX;
    motor->target_speed = vel_passos;
    
    if ((motor->target_speed * motor->current_speed < 0) || motor->target_speed == 0) {
        motor->acceleration = ACELERACAO_PARADA;
    } else { 
        motor->acceleration = ACELERACAO; 
    }
}

void Parar_Robo(void) {
    Aplicar_Movimento(&motorFL, 0.0f); Aplicar_Movimento(&motorFR, 0.0f);
    Aplicar_Movimento(&motorRL, 0.0f); Aplicar_Movimento(&motorRR, 0.0f);
}

// Processamento de rampas (1 kHz)
void Processar_Rampas_1kHz(void) {
    const float dt = 0.001f;
    StepperMotor* motores[] = {&motorFL, &motorFR, &motorRL, &motorRR};
    
    for (int i = 0; i < 4; i++) {
        StepperMotor* m = motores[i];
        float max_delta = m->acceleration * dt;
        
        if (fabsf(m->target_speed - m->current_speed) <= max_delta) {
            m->current_speed = m->target_speed;
        } else if (m->target_speed > m->current_speed) {
            m->current_speed += max_delta;
        } else {
            m->current_speed -= max_delta;
        }

        if (m->current_speed > 0.0f) {
            if (m->direction != 1) {
                HAL_GPIO_WritePin(m->port_dir, m->pin_dir, m->invert_dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
                m->direction = 1;
            }
        } else if (m->current_speed < 0.0f) {
            if (m->direction != -1) {
                HAL_GPIO_WritePin(m->port_dir, m->pin_dir, m->invert_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
                m->direction = -1;
            }
        } else {
            m->direction = 0;
        }

        m->step_increment = (uint32_t)(fabsf(m->current_speed) * FASE_POR_HZ);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        GPIOB->BRR = GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_13 | GPIO_PIN_15;

        StepperMotor* motores[] = {&motorFL, &motorFR, &motorRL, &motorRR};
        for(int i = 0; i < 4; i++) {
            StepperMotor* m = motores[i];
            uint32_t anterior = m->step_phase;
            m->step_phase = anterior + m->step_increment;
            if (m->step_phase < anterior) {
                m->port_pul->BSRR = m->pin_pul;
            }
        }

        static uint8_t divisor_rampa = 0;
        if (++divisor_rampa >= 50) {
            divisor_rampa = 0;
            Processar_Rampas_1kHz();
        }
    }
}

// Recuperação Inteligente de Barramento I2C
void I2C_Recuperar_Barramento(MPU_State* imu) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint16_t pin_scl, pin_sda;
    I2C_TypeDef* i2c_inst;

    if (imu->hi2c->Instance == I2C1) {
        pin_scl = GPIO_PIN_8; pin_sda = GPIO_PIN_9;
        i2c_inst = I2C1;
        __HAL_RCC_I2C1_CLK_ENABLE();
    } else {
        pin_scl = GPIO_PIN_10; pin_sda = GPIO_PIN_11;
        i2c_inst = I2C2;
        __HAL_RCC_I2C2_CLK_ENABLE();
    }

    HAL_I2C_DeInit(imu->hi2c);
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Pinos como saída open-drain para pulso manual
    HAL_GPIO_WritePin(GPIOB, pin_scl | pin_sda, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = pin_scl | pin_sda;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    for (volatile int j = 0; j < 200; j++) {}

    // 9 pulsos de clock para liberar o SDA
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, pin_scl, GPIO_PIN_RESET); for (volatile int j = 0; j < 200; j++) {}
        HAL_GPIO_WritePin(GPIOB, pin_scl, GPIO_PIN_SET);   for (volatile int j = 0; j < 200; j++) {}
    }

    // START + STOP manual
    HAL_GPIO_WritePin(GPIOB, pin_sda, GPIO_PIN_RESET); for (volatile int j = 0; j < 200; j++) {}
    HAL_GPIO_WritePin(GPIOB, pin_scl, GPIO_PIN_RESET); for (volatile int j = 0; j < 200; j++) {}
    HAL_GPIO_WritePin(GPIOB, pin_scl, GPIO_PIN_SET);   for (volatile int j = 0; j < 200; j++) {}
    HAL_GPIO_WritePin(GPIOB, pin_sda, GPIO_PIN_SET);   for (volatile int j = 0; j < 200; j++) {}

    // Retorna pinos para Alternate Function Open Drain
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    i2c_inst->CR1 |= I2C_CR1_SWRST; 
    for (volatile int j = 0; j < 200; j++) {}
    i2c_inst->CR1 &= ~I2C_CR1_SWRST;
    
    HAL_I2C_Init(imu->hi2c);
    imu_recuperacoes++;
}

bool MPU6050_Configurar(MPU_State* imu) {
    uint8_t d[2];
    if (HAL_I2C_Mem_Read(imu->hi2c, imu->i2c_addr, WHO_AM_I_REG, 1, &imu->whoami, 1, 50) != HAL_OK) {
        imu->whoami = 0;
        return false;
    }

    d[0] = PWR_MGMT_1_REG; d[1] = 0x00; HAL_I2C_Master_Transmit(imu->hi2c, imu->i2c_addr, d, 2, 50);
    d[0] = SMPLRT_DIV_REG; d[1] = 0x07; HAL_I2C_Master_Transmit(imu->hi2c, imu->i2c_addr, d, 2, 50);
    d[0] = CONFIG_REG;     d[1] = 0x03; HAL_I2C_Master_Transmit(imu->hi2c, imu->i2c_addr, d, 2, 50);
    d[0] = GYRO_CONFIG_REG;d[1] = 0x08; HAL_I2C_Master_Transmit(imu->hi2c, imu->i2c_addr, d, 2, 50);
    d[0] = ACCEL_CONFIG_REG;d[1]= 0x08; HAL_I2C_Master_Transmit(imu->hi2c, imu->i2c_addr, d, 2, 50);
    return true;
}

void MPU6050_Calibrar(MPU_State* imu) {
    float sum_gz = 0.0f;
    uint8_t raw[14];
    int amostras = 0;
    for (int i = 0; i < 200; i++) {
        uint8_t reg = ACCEL_XOUT_H_REG;
        if (HAL_I2C_Master_Transmit(imu->hi2c, imu->i2c_addr, &reg, 1, 20) == HAL_OK &&
            HAL_I2C_Master_Receive(imu->hi2c, imu->i2c_addr, raw, 14, 20) == HAL_OK) {
            int16_t gz_raw = (int16_t)(raw[12] << 8 | raw[13]);
            sum_gz += (gz_raw / 65.5f) * (PI / 180.0f);
            amostras++;
        }
        HAL_Delay(5);
    }
    imu->gyro_z_bias = amostras > 0 ? sum_gz / amostras : 0.0f;
}

void MPU6050_Init(void) {
    HAL_Delay(100);
    
    MPU_State* imus[] = {&imuFront, &imuRear};

    for (int i = 0; i < 2; i++) {
        imus[i]->presente = MPU6050_Configurar(imus[i]);
        if (!imus[i]->presente) {
            I2C_Recuperar_Barramento(imus[i]);
            imus[i]->presente = MPU6050_Configurar(imus[i]);
        }
    }

    // IMU ausente não é calibrada nem lida: evita ~9 s de timeouts no boot
    for (int i = 0; i < 2; i++) {
        if (imus[i]->presente) MPU6050_Calibrar(imus[i]);
    }
}

bool MPU6050_Read(MPU_State* imu, float dt) {
    uint8_t raw[14];
    uint8_t reg = ACCEL_XOUT_H_REG;
    
    if (HAL_I2C_Master_Transmit(imu->hi2c, imu->i2c_addr, &reg, 1, 5) != HAL_OK ||
        HAL_I2C_Master_Receive(imu->hi2c, imu->i2c_addr, raw, 14, 5) != HAL_OK) {
        
        imu->gx = 0.0f; imu->gy = 0.0f; imu->gz = 0.0f;
        imu->falhas++;

        if (imu->falhas >= 25) {
            imu->falhas = 0;
            I2C_Recuperar_Barramento(imu);
            MPU6050_Configurar(imu);
        }
        return false;
    }

    imu->falhas = 0;

    imu->ax = ((int16_t)(raw[0] << 8 | raw[1]) / 8192.0f) * 9.80665f;
    imu->ay = ((int16_t)(raw[2] << 8 | raw[3]) / 8192.0f) * 9.80665f;
    imu->az = ((int16_t)(raw[4] << 8 | raw[5]) / 8192.0f) * 9.80665f;
    imu->gx = ((int16_t)(raw[8] << 8 | raw[9]) / 65.5f) * (PI / 180.0f);
    imu->gy = ((int16_t)(raw[10] << 8 | raw[11]) / 65.5f) * (PI / 180.0f);
    imu->gz = (((int16_t)(raw[12] << 8 | raw[13]) / 65.5f) * (PI / 180.0f)) - imu->gyro_z_bias;

    imu->roll = atan2f(imu->ay, imu->az);
    imu->pitch = atan2f(-imu->ax, sqrtf(imu->ay * imu->ay + imu->az * imu->az));

    imu->yaw += imu->gz * dt;
    if (imu->yaw > PI) imu->yaw -= 2.0f * PI;
    else if (imu->yaw < -PI) imu->yaw += 2.0f * PI;

    return true;
}

// Transmissão para o nó ROS
static bool Formatar_Valor_Fixo(char* destino, size_t tamanho, float valor,
                                uint32_t escala, unsigned int casas_decimais) {
    if (!isfinite(valor) || escala == 0) return false;

    float valor_escalado = valor * (float)escala;
    if (valor_escalado >= 2147483000.0f || valor_escalado <= -2147483000.0f) return false;

    int32_t valor_inteiro = (int32_t)lroundf(valor_escalado);
    int64_t magnitude = valor_inteiro;
    if (magnitude < 0) magnitude = -magnitude;
    int escrito = snprintf(destino, tamanho, "%s%lu.%0*lu",
                           valor_inteiro < 0 ? "-" : "",
                           (unsigned long)((uint64_t)magnitude / escala),
                           (int)casas_decimais,
                           (unsigned long)((uint64_t)magnitude % escala));
    return escrito > 0 && (size_t)escrito < tamanho;
}

extern USBD_HandleTypeDef hUsbDeviceFS;

static bool USB_Livre(void) {
    USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    return hcdc != NULL && hcdc->TxState == 0;
}

void Publicar_Status_IMU_USB(void) {
    if (!USB_Livre()) return;

    static char msg[128];
    int len = snprintf(msg, sizeof(msg),
                       "STATUS:F_who=0x%02X,F_fail=%lu,R_who=0x%02X,R_fail=%lu,rec=%lu\n",
                       imuFront.whoami, (unsigned long)imuFront.falhas,
                       imuRear.whoami, (unsigned long)imuRear.falhas,
                       (unsigned long)imu_recuperacoes);
    if (len > 0 && (size_t)len < sizeof(msg)) CDC_Transmit_FS((uint8_t*)msg, (uint16_t)len);
}

void Publicar_IMU_USB(MPU_State* imu) {
    if (!USB_Livre()) return;

    float cy = cosf(imu->yaw * 0.5f);
    float sy = sinf(imu->yaw * 0.5f);
    float cp = cosf(imu->pitch * 0.5f);
    float sp = sinf(imu->pitch * 0.5f);
    float cr = cosf(imu->roll * 0.5f);
    float sr = sinf(imu->roll * 0.5f);

    float qw = cr * cp * cy + sr * sp * sy;
    float qx = sr * cp * cy - cr * sp * sy;
    float qy = cr * sp * cy + sr * cp * sy;
    float qz = cr * cp * sy - sr * sp * cy;

    const float valores[10] = {qx, qy, qz, qw, imu->gx, imu->gy, imu->gz, imu->ax, imu->ay, imu->az};
    char campos[10][16];
    for (unsigned int i = 0; i < 10; i++) {
        uint32_t escala = i < 7 ? 10000U : 100U;
        unsigned int casas_decimais = i < 7 ? 4U : 2U;
        if (!Formatar_Valor_Fixo(campos[i], sizeof(campos[i]), valores[i],
                                 escala, casas_decimais)) {
            return;
        }
    }

    static char msg[128];
    int len = snprintf(msg, sizeof(msg),
                       "%s:%s,%s,%s,%s,%s,%s,%s,%s,%s,%s;\n",
                       imu->prefix, campos[0], campos[1], campos[2], campos[3], campos[4],
                       campos[5], campos[6], campos[7], campos[8], campos[9]);
    if (len > 0 && (size_t)len < sizeof(msg)) {
        CDC_Transmit_FS((uint8_t*)msg, (uint16_t)len);
    }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_I2C1_Init(); // Inicia barramento I2C1 (Front)
  MX_I2C2_Init(); // Inicia barramento I2C2 (Rear)
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  Motores_Init();
  MPU6050_Init();
  HAL_TIM_Base_Start_IT(&htim3);

  // Escalona as IMUs em 10 ms: nunca são lidas na mesma passagem do loop
  imuFront.proxima_leitura = HAL_GetTick() + 20;
  imuRear.proxima_leitura  = HAL_GetTick() + 30;
  uint32_t last_status_time = HAL_GetTick();
  /* USER CODE END 2 */

  while (1)
  {
    if (flag_comando_recebido) {
        flag_comando_recebido = false;

        char* tokFL = strtok((char*)rx_buffer, ",");
        char* tokFR = tokFL ? strtok(NULL, ",") : NULL;
        char* tokRL = tokFR ? strtok(NULL, ",") : NULL;
        char* tokRR = tokRL ? strtok(NULL, ",") : NULL;

        if (tokFL && tokFR && tokRL && tokRR) {
            float vFL = atof(tokFL), vFR = atof(tokFR), vRL = atof(tokRL), vRR = atof(tokRR);

            if (!isfinite(vFL) || !isfinite(vFR) || !isfinite(vRL) || !isfinite(vRR)) {
                continue;
            }

            ultimoComandoValido = HAL_GetTick();

            if (fabs(vFL - ultimoVFL) > EPSILON_VEL || fabs(vFR - ultimoVFR) > EPSILON_VEL ||
                fabs(vRL - ultimoVRL) > EPSILON_VEL || fabs(vRR - ultimoVRR) > EPSILON_VEL) {

                if (vFL == 0.0f && vFR == 0.0f && vRL == 0.0f && vRR == 0.0f) {
                    Parar_Robo(); roboParado = true;
                } else {
                    Aplicar_Movimento(&motorFL, vFL);
                    Aplicar_Movimento(&motorFR, vFR);
                    Aplicar_Movimento(&motorRL, vRL);
                    Aplicar_Movimento(&motorRR, vRR);
                    roboParado = false;
                }
                ultimoVFL = vFL; ultimoVFR = vFR; ultimoVRL = vRL; ultimoVRR = vRR;
            }
        }
    }

    if (!roboParado && (HAL_GetTick() - ultimoComandoValido > SERIAL_TIMEOUT_MS)) {
        Parar_Robo(); roboParado = true;
        ultimoVFL = 0.0f; ultimoVFR = 0.0f; ultimoVRL = 0.0f; ultimoVRR = 0.0f;
    }

    uint32_t now = HAL_GetTick();
    MPU_State* imus[] = {&imuFront, &imuRear};
    for (int i = 0; i < 2; i++) {
        MPU_State* imu = imus[i];
        if (!imu->presente || (int32_t)(now - imu->proxima_leitura) < 0) continue;

        float dt = (now - imu->ultima_leitura) / 1000.0f;
        if (dt > 0.1f) dt = 0.02f;

        imu->ultima_leitura = now;
        imu->proxima_leitura = now + 20;

        if (MPU6050_Read(imu, dt)) {
            Publicar_IMU_USB(imu);
        }
    }

    if (now - last_status_time >= 1000) {
        last_status_time = now;
        Publicar_Status_IMU_USB();
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  // Ativa clock do AFIO para permitir o Remap dos pinos
  __HAL_RCC_AFIO_CLK_ENABLE();
  // Remapeia o I2C1 do PB6/PB7 para o PB8/PB9
  __HAL_AFIO_REMAP_I2C1_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|
                           GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|
                        GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configuração dos Pinos I2C1 Remap (PB8 SCL, PB9 SDA)
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configuração dos Pinos I2C2 (PB10 SCL, PB11 SDA)
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
