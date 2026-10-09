
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Atualizado para STM32F103C8T6 - Blue Pill)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define VELOCIDADE_MAX 4800.0f
#define ACELERACAO 14000.0f
#define ACELERACAO_PARADA 16000.0f
#define RAIO_RODA 0.05f
#define PULSOS_POR_REV 6400.0f
#define PI 3.14159265359f
#define STEPS_POR_METRO (PULSOS_POR_REV / (2.0f * PI * RAIO_RODA))
#define FREQ_TIMER 50000.0f // TIM3: Prescaler=71, Period=19 -> 1 MHz / 20 ticks = 50 kHz
#define FASE_POR_HZ (4294967296.0f / FREQ_TIMER) // 2^32 / FREQ_TIMER

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

// Interrupção: Recebe os dados do ROS via USB
void Receber_Dados_USB(uint8_t* Buf, uint32_t Len) {
    for(uint32_t i = 0; i < Len; i++) {
        if (Buf[i] == '\n') {
            rx_buffer[rx_index] = '\0';
            flag_comando_recebido = true;
            rx_index = 0;
            // Pisca o LED embutido da Blue Pill (PC13)
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        } else {
            if (rx_index < RX_BUFFER_SIZE - 1) rx_buffer[rx_index++] = Buf[i];
        }
    }
}

// Mapeamento PUL/DIR dos quatro motores no GPIOB
void Motores_Init(void) {
    // Frente Esquerda (FL): PUL = PB15, DIR = PB14
    motorFL.port_pul = GPIOB; motorFL.pin_pul = GPIO_PIN_15;
    motorFL.port_dir = GPIOB; motorFL.pin_dir = GPIO_PIN_14; motorFL.invert_dir = true;

    // Frente Direita (FR): PUL = PB6, DIR = PB5
    motorFR.port_pul = GPIOB; motorFR.pin_pul = GPIO_PIN_6;
    motorFR.port_dir = GPIOB; motorFR.pin_dir = GPIO_PIN_5; motorFR.invert_dir = false;

    // Atrás Esquerda (RL): PUL = PB13, DIR = PB12
    motorRL.port_pul = GPIOB; motorRL.pin_pul = GPIO_PIN_13;
    motorRL.port_dir = GPIOB; motorRL.pin_dir = GPIO_PIN_12; motorRL.invert_dir = true;

    // Atrás Direita (RR): PUL = PB3, DIR = PB4
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
    } else { motor->acceleration = ACELERACAO; }
}

void Parar_Robo(void) {
    Aplicar_Movimento(&motorFL, 0.0f); Aplicar_Movimento(&motorFR, 0.0f);
    Aplicar_Movimento(&motorRL, 0.0f); Aplicar_Movimento(&motorRR, 0.0f);
}

void Processar_Rampas(void) {
    float dt = 0.001f;
    StepperMotor* motores[] = {&motorFL, &motorFR, &motorRL, &motorRR};
    for (int i = 0; i < 4; i++) {
        StepperMotor* m = motores[i];
        float max_delta = m->acceleration * dt;
        if (fabs(m->target_speed - m->current_speed) <= max_delta) m->current_speed = m->target_speed;
        else if (m->target_speed > m->current_speed) m->current_speed += max_delta;
        else m->current_speed -= max_delta;

        if (m->current_speed > 0) {
            m->direction = 1; HAL_GPIO_WritePin(m->port_dir, m->pin_dir, m->invert_dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
        } else if (m->current_speed < 0) {
            m->direction = -1; HAL_GPIO_WritePin(m->port_dir, m->pin_dir, m->invert_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
        } else m->direction = 0;

        // Incremento de fase do acumulador da interrupção (atualizado depois do DIR).
        m->step_increment = (uint32_t)(fabsf(m->current_speed) * FASE_POR_HZ);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        // Desliga os pinos de pulso dos motores simultaneamente.
        GPIOB->BRR = GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_13 | GPIO_PIN_15;

        // Acumulador de fase em inteiros (sem float): cada estouro de 32 bits gera um pulso.
        StepperMotor* motores[] = {&motorFL, &motorFR, &motorRL, &motorRR};

        for(int i = 0; i < 4; i++) {
            StepperMotor* m = motores[i];
            uint32_t anterior = m->step_phase;
            m->step_phase = anterior + m->step_increment;
            if (m->step_phase < anterior) m->port_pul->BSRR = m->pin_pul;
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  Motores_Init();

  HAL_TIM_Base_Start_IT(&htim3);
  uint32_t last_ramp_time = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (flag_comando_recebido) {
        flag_comando_recebido = false;

        char* tokFL = strtok((char*)rx_buffer, ",");
        char* tokFR = tokFL ? strtok(NULL, ",") : NULL;
        char* tokRL = tokFR ? strtok(NULL, ",") : NULL;
        char* tokRR = tokRL ? strtok(NULL, ",") : NULL;

        if (tokFL && tokFR && tokRL && tokRR) {
            float vFL = atof(tokFL), vFR = atof(tokFR), vRL = atof(tokRL), vRR = atof(tokRR);
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

    if (HAL_GetTick() - last_ramp_time >= 1) {
        Processar_Rampas(); last_ramp_time = HAL_GetTick();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /* Configure motor GPIO output levels */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|
                           GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /* Configure GPIO pin : PC13 (LED Interno) */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure motor GPIO pins: PB3-PB6 and PB12-PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|
                        GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure GPIO pin : PB0 (Botao Emergencia) */
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
#endif /* USE_FULL_ASSERT */
