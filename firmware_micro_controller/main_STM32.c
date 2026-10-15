/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F103C8T6 - Controle 4 Motores + MPU6050 (I2C2)
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
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// Registradores da MPU6050
#define MPU6050_ADDR         0xD0
#define SMPLRT_DIV_REG       0x19
#define CONFIG_REG           0x1A
#define GYRO_CONFIG_REG      0x1B
#define ACCEL_CONFIG_REG     0x1C
#define PWR_MGMT_1_REG       0x6B
#define ACCEL_XOUT_H_REG     0x3B
#define WHO_AM_I_REG         0x75

// Variáveis da IMU
float ax = 0.0f, ay = 0.0f, az = 0.0f;
float gx = 0.0f, gy = 0.0f, gz = 0.0f;
float gyro_z_bias = 0.0f;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

// Diagnóstico da IMU (enviado periodicamente em linhas IMU_STATUS)
uint8_t imu_whoami = 0;
uint32_t imu_falhas = 0;
uint32_t imu_erro_i2c = 0;
uint32_t imu_recuperacoes = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN 0 */
#define PULSOS_POR_REV 1600.0f
#define VELOCIDADE_MAX 4000.0f
#define ACELERACAO 4500.0f
#define ACELERACAO_PARADA 5000.0f
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

        // [REVISÃO] Acesso condicional aos pinos para poupar processamento
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

// Recuperação do I2C2 (errata STM32F10xx 2.13.7: flag BUSY preso com o barramento livre)
static void I2C_Atraso(void) {
    for (volatile int i = 0; i < 200; i++) {}
}

void I2C2_Recuperar_Barramento(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_I2C_DeInit(&hi2c2);
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // PB10 (SCL) e PB11 (SDA) como saída open-drain em nível alto
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    I2C_Atraso();

    // 9 pulsos de clock para liberar um escravo que esteja segurando o SDA
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); I2C_Atraso();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   I2C_Atraso();
    }

    // Sequência START + STOP manual (SDA baixo, SCL baixo, SCL alto, SDA alto)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); I2C_Atraso();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); I2C_Atraso();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   I2C_Atraso();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   I2C_Atraso();

    // Reset por software do periférico; HAL_I2C_Init volta os pinos para AF open-drain
    __HAL_RCC_I2C2_CLK_ENABLE();
    I2C2->CR1 |= I2C_CR1_SWRST;
    I2C_Atraso();
    I2C2->CR1 &= ~I2C_CR1_SWRST;
    HAL_I2C_Init(&hi2c2);

    imu_recuperacoes++;
}

// Configuração dos registradores da MPU6050; retorna false se o sensor não responder
bool MPU6050_Configurar(void) {
    uint8_t d[2];

    // Esperado 0x68. 0x00 = sem resposta na I2C (fiação, pull-up, alimentação ou AD0 em nível alto)
    if (HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT,
                         &imu_whoami, 1, 100) != HAL_OK) {
        imu_whoami = 0;
        imu_erro_i2c = hi2c2.ErrorCode;
        return false;
    }

    d[0] = PWR_MGMT_1_REG; d[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c2, MPU6050_ADDR, d, 2, 100);

    d[0] = SMPLRT_DIV_REG; d[1] = 0x07;
    HAL_I2C_Master_Transmit(&hi2c2, MPU6050_ADDR, d, 2, 100);

    d[0] = CONFIG_REG; d[1] = 0x03;
    HAL_I2C_Master_Transmit(&hi2c2, MPU6050_ADDR, d, 2, 100);

    d[0] = GYRO_CONFIG_REG; d[1] = 0x08;
    HAL_I2C_Master_Transmit(&hi2c2, MPU6050_ADDR, d, 2, 100);

    d[0] = ACCEL_CONFIG_REG; d[1] = 0x08;
    HAL_I2C_Master_Transmit(&hi2c2, MPU6050_ADDR, d, 2, 100);
    return true;
}

// Inicialização da IMU
void MPU6050_Init(void) {
    HAL_Delay(100);

    if (!MPU6050_Configurar()) {
        I2C2_Recuperar_Barramento();
        if (!MPU6050_Configurar()) return;
    }

    // Calibração do Gyro Z
    float sum_gz = 0.0f;
    uint8_t raw[14];
    int amostras = 0;
    for (int i = 0; i < 200; i++) {
        uint8_t reg = ACCEL_XOUT_H_REG;
        if (HAL_I2C_Master_Transmit(&hi2c2, MPU6050_ADDR, &reg, 1, 50) == HAL_OK &&
            HAL_I2C_Master_Receive(&hi2c2, MPU6050_ADDR, raw, 14, 50) == HAL_OK) {
            int16_t gz_raw = (int16_t)(raw[12] << 8 | raw[13]);
            sum_gz += (gz_raw / 65.5f) * (PI / 180.0f);
            amostras++;
        }
        HAL_Delay(5);
    }
    gyro_z_bias = amostras > 0 ? sum_gz / amostras : 0.0f;
}

// Leitura da MPU6050
bool MPU6050_Read(float dt) {
    static uint8_t falhas_seguidas = 0;
    uint8_t raw[14];
    uint8_t reg = ACCEL_XOUT_H_REG;
    
    // [REVISÃO] Se der falha de comunicação, zera as velocidades para não sujar o ROS
    if (HAL_I2C_Master_Transmit(&hi2c2, MPU6050_ADDR, &reg, 1, 10) != HAL_OK ||
        HAL_I2C_Master_Receive(&hi2c2, MPU6050_ADDR, raw, 14, 10) != HAL_OK) {
        gx = 0.0f; gy = 0.0f; gz = 0.0f;
        imu_falhas++;
        imu_erro_i2c = hi2c2.ErrorCode;

        // Após ~0.5 s de falhas seguidas, tenta destravar o barramento e reconfigurar o sensor
        if (++falhas_seguidas >= 25) {
            falhas_seguidas = 0;
            I2C2_Recuperar_Barramento();
            MPU6050_Configurar();
        }
        return false;
    }

    falhas_seguidas = 0;

    int16_t raw_ax = (int16_t)(raw[0] << 8 | raw[1]);
    int16_t raw_ay = (int16_t)(raw[2] << 8 | raw[3]);
    int16_t raw_az = (int16_t)(raw[4] << 8 | raw[5]);
    int16_t raw_gx = (int16_t)(raw[8] << 8 | raw[9]);
    int16_t raw_gy = (int16_t)(raw[10] << 8 | raw[11]);
    int16_t raw_gz = (int16_t)(raw[12] << 8 | raw[13]);

    ax = (raw_ax / 8192.0f) * 9.80665f;
    ay = (raw_ay / 8192.0f) * 9.80665f;
    az = (raw_az / 8192.0f) * 9.80665f;

    gx = (raw_gx / 65.5f) * (PI / 180.0f);
    gy = (raw_gy / 65.5f) * (PI / 180.0f);
    gz = ((raw_gz / 65.5f) * (PI / 180.0f)) - gyro_z_bias;

    // [REVISÃO] Cálculo real de Roll e Pitch baseado na gravidade
    roll = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    // Integração do Yaw
    yaw += gz * dt;
    
    // [REVISÃO] Trava matemática: Mantém o Yaw estritamente entre -PI e +PI radianos
    if (yaw > PI) yaw -= 2.0f * PI;
    else if (yaw < -PI) yaw += 2.0f * PI;
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

// CDC_Transmit_FS é assíncrono: o buffer precisa continuar válido até o fim
// da transferência (mensagens > 64 bytes vão em mais de um pacote USB).
// Se a transmissão anterior ainda está em andamento, a mensagem é descartada.
static bool USB_Livre(void) {
    USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    return hcdc != NULL && hcdc->TxState == 0;
}

void Publicar_Status_IMU_USB(void) {
    if (!USB_Livre()) return;

    static char msg[128];
    int len = snprintf(msg, sizeof(msg),
                       "IMU_STATUS:whoami=0x%02X,falhas=%lu,erro_i2c=0x%02lX,rec=%lu,busy=%d,scl_pb10=%d,sda_pb11=%d\n",
                       imu_whoami, (unsigned long)imu_falhas, (unsigned long)imu_erro_i2c,
                       (unsigned long)imu_recuperacoes, (I2C2->SR2 & I2C_SR2_BUSY) != 0,
                       // Nível real dos pinos; em repouso o barramento I2C deve ficar em 1
                       HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET,
                       HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET);
    if (len <= 0 || (size_t)len >= sizeof(msg)) return;

    CDC_Transmit_FS((uint8_t*)msg, (uint16_t)len);
}

void Publicar_IMU_USB(void) {
    if (!USB_Livre()) return;

    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    float qw = cr * cp * cy + sr * sp * sy;
    float qx = sr * cp * cy - cr * sp * sy;
    float qy = cr * sp * cy + sr * cp * sy;
    float qz = cr * cp * sy - sr * sp * cy;

    const float valores[10] = {qx, qy, qz, qw, gx, gy, gz, ax, ay, az};
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
                       "IMU:%s,%s,%s,%s,%s,%s,%s,%s,%s,%s;\n",
                       campos[0], campos[1], campos[2], campos[3], campos[4],
                       campos[5], campos[6], campos[7], campos[8], campos[9]);
    if (len <= 0 || (size_t)len >= sizeof(msg)) return;

    CDC_Transmit_FS((uint8_t*)msg, (uint16_t)len);
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  Motores_Init();
  MPU6050_Init();
  HAL_TIM_Base_Start_IT(&htim3);

  uint32_t last_imu_time = HAL_GetTick();
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
    if (now - last_imu_time >= 20) {
        float dt = (now - last_imu_time) / 1000.0f;
        
        // [REVISÃO] Proteção: impede saltos absurdos caso o loop atrase
        if (dt > 0.1f) dt = 0.02f; 
        
        last_imu_time = now;
        // Só publica leituras reais; sem dados, o ROS detecta a IMU como inválida
        if (MPU6050_Read(dt)) {
            Publicar_IMU_USB();
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

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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