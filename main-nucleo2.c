/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "notas.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIM_FREQ 60000000
#define ARR 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile bool shotRequested       = false;
// Nuevo: disparo enemigo
volatile bool enemyShotRequested  = false;
// Nuevo: explosión
volatile bool explosionRequested  = false;
// Para comandos numéricos (UART1)
volatile bool loopImperial = false;
uint32_t impoStart = 0;               // timestamp de inicio de la nota actual
uint32_t impoRemain = 0;              // ms restantes de la nota al pausar
int      impoIdx    = 0;              // índice en ImperialMarchMelody/Durations
bool     impoPlaying= false;          // si la nota está sonando
// Estado de disparo
const uint32_t shotFreq     = 880;    // Hz del disparo
const uint32_t shotDur      = 100;    // ms de disparo


volatile char  cmdNum       = 0;
volatile bool  cmdNumReady  = false;

// Para comandos de letra (UART2)
volatile char  cmdLet       = 0;
volatile bool  cmdLetReady  = false;
volatile bool loopFein       = false;
volatile bool stop_request   = false;
uint8_t rxByte;
volatile uint8_t cmd = 0;
volatile bool cmd_ready = false;
uint8_t rx1, rx2, rx3;

int notes[]= {NOTE_E5, NOTE_E5, NOTE_E5,
	    NOTE_C5, NOTE_E5,
	    NOTE_G5, NOTE_G4,
	    NOTE_C5, NOTE_G4, NOTE_E4,
	    NOTE_A4, NOTE_B4, NOTE_AS4, NOTE_A4,
	    NOTE_G4, NOTE_E5, NOTE_G5, NOTE_A5,
	    NOTE_F5, NOTE_G5, NOTE_E5, NOTE_C5, NOTE_D5, NOTE_B4};
int duration[]= { 75000, 75000, 75000,
	    75000, 15000,
	    150000, 150000,
	    75000, 75000, 75000,
	    150000, 150000, 37500, 37500,
	    75000, 75000, 75000, 150000,
	    75000, 75000, 75000, 75000, 75000, 150000 };
int pauses[]= {50, 50, 50,
	    50, 50,
	    100, 100,
	    50, 50, 50,
	    50, 50, 25, 25,
	    50, 50, 50, 50,
	    50, 50, 50, 50, 50, 100};
int Mariomelody[] = {
659, 659, 659, 0, 523, 659, 784, 392, 523, 0, 392, 0, 330, 0, 440, 0, 494, 0, 466, 440, 392, 659, 784, 880, 698, 784, 0, 659, 0, 523, 587, 494, 0, 523, 0, 392, 0, 330, 0, 440, 0, 494, 0, 466, 440, 392, 659, 784, 880, 698, 784, 0, 659, 0, 523, 587, 494, 0, 0, 784, 740, 698, 622, 659, 0, 415, 440, 523, 0, 440, 523, 587, 0, 784, 740, 698, 622, 659, 0, 698, 1047, 0, 698, 1047, 698, 1047, 0, 784, 740, 698, 622, 659, 0, 415, 440, 523, 0, 440, 523, 587, 0, 622, 0, 587, 0, 523, 0, 0, 784, 740, 698, 622, 659, 0, 415, 440, 523, 0, 440, 523, 587, 0, 784, 740, 698, 622, 659, 0, 698, 1047, 0, 698, 1047, 698, 1047, 0, 784, 740, 698, 622, 659, 0, 415, 440, 523, 0, 440, 523, 587, 0, 622, 0, 587, 0, 523, 0, 523, 523, 523, 0, 523, 587, 659, 523, 440, 392, 523, 523, 523, 0, 523, 587, 659, 0, 523, 523, 523, 0, 523, 587, 659, 523, 440, 392, 659, 659, 659, 0, 523, 659, 784, 392, 523, 0, 392, 0, 330, 0, 440, 0, 494, 0, 466, 440, 392, 659, 784, 880, 698, 784, 0, 659, 0, 523, 587, 494, 0, 523, 0, 392, 0, 330, 0, 440, 0, 494, 0, 466, 440, 392, 659, 784, 880, 698, 784, 0, 659, 0, 523, 587, 494, 0, 659, 523, 392, 0, 415, 440, 698, 698, 440, 494, 880, 880, 880, 784, 698, 659, 523, 440, 392, 659, 523, 392, 0, 415, 440, 698, 698, 440, 494, 698, 698, 698, 659, 587, 523, 392, 392, 262, 523, 523, 523, 0, 523, 587, 659, 523, 440, 392, 523, 523, 523, 0, 523, 587, 659, 0, 523, 523, 523, 0, 523, 587, 659, 523, 440, 392, 659, 659, 659, 0, 523, 659, 784, 392, 659, 523, 392, 0, 415, 440, 698, 698, 440, 494, 880, 880, 880, 784, 698, 659, 523, 440, 392, 659, 523, 392, 0, 415, 440, 698, 698, 440, 494, 698, 698, 698, 659, 587, 523, 392, 392, 262, 0
};
int MarionoteDurations[] = {
126, 252, 126, 126, 126, 252, 504, 504, 252, 126, 126, 252, 252, 126, 126, 126, 126, 126, 126, 252, 168, 168, 168, 252, 126, 126, 126, 126, 126, 126, 126, 126, 252, 252, 126, 126, 252, 252, 126, 126, 126, 126, 126, 126, 252, 168, 168, 168, 252, 126, 126, 126, 126, 126, 126, 126, 126, 252, 252, 126, 126, 126, 252, 126, 126, 126, 126, 126, 126, 126, 126, 126, 252, 126, 126, 126, 252, 126, 126, 126, 126, 126, 126, 126, 504, 504, 252, 126, 126, 126, 252, 126, 126, 126, 126, 126, 126, 126, 126, 126, 252, 252, 126, 126, 252, 504, 504, 252, 126, 126, 126, 252, 126, 126, 126, 126, 126, 126, 126, 126, 126, 252, 126, 126, 126, 252, 126, 126, 126, 126, 126, 126, 126, 504, 504, 252, 126, 126, 126, 252, 126, 126, 126, 126, 126, 126, 126, 126, 126, 252, 252, 126, 126, 252, 504, 504, 126, 252, 126, 126, 126, 252, 126, 252, 126, 504, 126, 252, 126, 126, 126, 126, 126, 1008, 126, 252, 126, 126, 126, 252, 126, 252, 126, 504, 126, 252, 126, 126, 126, 252, 504, 504, 252, 126, 126, 252, 252, 126, 126, 126, 126, 126, 126, 252, 168, 168, 168, 252, 126, 126, 126, 126, 126, 126, 126, 126, 252, 252, 126, 126, 252, 252, 126, 126, 126, 126, 126, 126, 252, 168, 168, 168, 252, 126, 126, 126, 126, 126, 126, 126, 126, 252, 126, 252, 126, 252, 252, 126, 252, 126, 504, 168, 168, 168, 168, 168, 168, 126, 252, 126, 504, 126, 252, 126, 252, 252, 126, 252, 126, 504, 126, 252, 126, 168, 168, 168, 126, 252, 126, 504, 126, 252, 126, 126, 126, 252, 126, 252, 126, 504, 126, 252, 126, 126, 126, 126, 126, 1008, 126, 252, 126, 126, 126, 252, 126, 252, 126, 504, 126, 252, 126, 126, 126, 252, 504, 504, 126, 252, 126, 252, 252, 126, 252, 126, 504, 168, 168, 168, 168, 168, 168, 126, 252, 126, 504, 126, 252, 126, 252, 252, 126, 252, 126, 504, 126, 252, 126, 168, 168, 168, 126, 252, 126, 504, 0
};
int GalagaNotes[] = {
    NOTE_B6, 0, NOTE_E7, NOTE_DS5, NOTE_B4, 0, NOTE_FS5, NOTE_DS5,
    NOTE_E5, 0, NOTE_FS5, NOTE_E5, NOTE_DS5, NOTE_B4, NOTE_DS5, NOTE_E5,
    NOTE_FS5, NOTE_E5, NOTE_DS5, NOTE_B4, NOTE_DS5, NOTE_E5, NOTE_FS5, NOTE_E5
};
int Feindurations[] = {
504, 504, 252, 252, 504, 126, 126, 126, 126, 126, 126, 252, 252, 252, 504, 504, 504, 252, 252, 504, 126, 126, 126, 126, 126, 126, 252, 252, 252, 504, 504, 504, 504, 504, 252, 252, 252, 252, 378, 126, 252, 252, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 504, 504, 504, 378, 126, 504, 504, 168, 168, 168, 168, 168, 168, 504, 504, 504, 378, 126, 504, 504, 168, 168, 168, 168, 168, 168, 504, 504, 504, 504, 504, 504, 252, 252, 252, 252, 504, 504, 1008, 0
};
int Feinmelody[] = {
349, 349, 415, 466, 466, 554, 523, 554, 523, 554, 523, 415, 415, 466, 466, 349, 349, 415, 466, 466, 554, 523, 554, 523, 554, 523, 415, 415, 466, 349, 349, 523, 415, 349, 554, 554, 554, 554, 622, 554, 523, 523, 466, 523, 554, 698, 831, 698, 554, 523, 466, 523, 554, 698, 831, 698, 554, 523, 466, 523, 554, 698, 831, 698, 554, 523, 466, 523, 554, 698, 831, 698, 554, 523, 311, 415, 523, 622, 831, 622, 523, 415, 311, 415, 523, 622, 831, 622, 523, 415, 311, 415, 523, 622, 831, 622, 523, 415, 311, 415, 523, 622, 831, 622, 523, 415, 466, 466, 466, 415, 440, 466, 466, 831, 880, 933, 622, 659, 698, 466, 466, 466, 415, 440, 466, 466, 831, 880, 933, 622, 659, 698, 415, 415, 415, 415, 415, 415, 622, 554, 523, 466, 466, 0, 0, 0
};

// Duraciones cuantizadas al tempo (ms)
int GalagaDurations[] = {
    // Patrón rítmico característico
    750, 250, 750, 250,
    500, 250, 500, 250,
    500, 250, 500, 250,
    500, 250, 500, 250,
    500, 250, 500, 250,
    500, 250, 500, 250
};
int GalagaShotMelody[] = {
    NOTE_B6,  // 1976 Hz
    NOTE_A6,  // 1760 Hz
    NOTE_G6,  // 1568 Hz
    NOTE_FS6, // 1480 Hz
    NOTE_F6,  // 1397 Hz
    NOTE_E6,  // 1319 Hz
    NOTE_DS6, // 1245 Hz
    NOTE_D6,  // 1175 Hz
    NOTE_CS6, // 1109 Hz
    NOTE_C6,  // 1047 Hz
    NOTE_B5,  //  988 Hz
    NOTE_AS5, //  932 Hz
    NOTE_A5,  //  880 Hz
    NOTE_GS5, //  831 Hz
    NOTE_G5   //  784 Hz
};
int GalagaShotDurations[] = {
    20, 20, 20, 20, 20,
    20, 20, 20, 20, 20,
    20, 20, 20, 20, 20
};

// La Marcha Imperial (Star Wars) – PWM tones @120 BPM
// Melody mapped to notas.h macros
// La Marcha Imperial – Star Wars (PWM tones @120 BPM)

// Melodía (dos frases principales)
int ImperialMarchMelody[] = {
    // Frase 1: G–G–G–Eb / Bb–G–Eb–Bb / G (larga)
    NOTE_G4,  NOTE_G4,  NOTE_G4,  NOTE_DS4,
    NOTE_AS4, NOTE_G4,  NOTE_DS4, NOTE_AS4,
    NOTE_G4,

    // Frase 2: D–D–D–Eb / Bb–G–Eb–Bb / G (larga)
    NOTE_D5,  NOTE_D5,  NOTE_D5,  NOTE_DS5,
    NOTE_AS4, NOTE_G4,  NOTE_DS4, NOTE_AS4,
    NOTE_G4
};

// Duraciones (ms) alineadas al tempo:
int ImperialMarchDurations[] = {
    // Frase 1
    500, 500, 500, 350,
    150, 500, 350, 150,
    1000,

    // Frase 2
    500, 500, 500, 350,
    150, 500, 350, 150,
    1000
};
int ImperialMarchNotes[] = {
    // Primer fraseo
    NOTE_G4, NOTE_G4, NOTE_G4, NOTE_DS4, NOTE_B4, NOTE_G4, NOTE_DS4, NOTE_B4,
    NOTE_G4, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_DS5, NOTE_B4, NOTE_GS4, NOTE_DS4,
    NOTE_B4, NOTE_G4, NOTE_GS4, NOTE_DS4, NOTE_GS4, NOTE_G4, 0,

    // Repetición
    NOTE_G4, NOTE_G4, NOTE_G4, NOTE_DS4, NOTE_B4, NOTE_G4, NOTE_DS4, NOTE_B4,
    NOTE_G4, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_DS5, NOTE_B4, NOTE_GS4, NOTE_DS4,
    NOTE_B4, NOTE_G4, NOTE_GS4, NOTE_DS4, NOTE_GS4, NOTE_G4, 0,

    // Final épico
    NOTE_DS5, NOTE_DS5, NOTE_DS5, NOTE_DS5, NOTE_DS5, NOTE_D5, NOTE_CS5, NOTE_C5,
    NOTE_B4, NOTE_AS4, NOTE_A4, NOTE_GS4, NOTE_G4, NOTE_FS4, NOTE_F4, NOTE_E4,
    NOTE_DS4, NOTE_D4, 0
};

#define PATTERN_LEN  (8*2 + 12*1)

// Duración en milisegundos (120 BPM = 500ms por negra)
int ImperialMarchDurations2[] = {
    // Valores en ms (patrón: corchea=250, negra=500, blanca=1000)
    350, 350, 350, 500, 250, 500, 250, 250,
    1000, 350, 350, 350, 500, 250, 500, 250,
    250, 1000, 500, 250, 500, 1000, 500,

    // Repetición
    350, 350, 350, 500, 250, 500, 250, 250,
    1000, 350, 350, 350, 500, 250, 500, 250,
    250, 1000, 500, 250, 500, 1000, 500,

    // Final
    150, 150, 150, 150, 150, 150, 150, 150,
    150, 150, 150, 150, 150, 150, 150, 150,
    500, 1000, 1000
};

const int PatternMelody[PATTERN_LEN] = {
  // Intro (8 notas)
  NOTE_AS4, NOTE_FS5, NOTE_DS5, NOTE_F5,
  NOTE_DS5, NOTE_CS5, NOTE_B4,  NOTE_AS4,
  // Intro again
  NOTE_AS4, NOTE_FS5, NOTE_DS5, NOTE_F5,
  NOTE_DS5, NOTE_CS5, NOTE_B4,  NOTE_AS4,

  // Chorus (12 notas)
  NOTE_DS5, NOTE_CS5, NOTE_B4,  NOTE_AS4,
  NOTE_G4,  NOTE_G4,  NOTE_AS4, NOTE_B4,
  NOTE_C5,  NOTE_C5,  NOTE_DS5, NOTE_F5
};


const int PatternDurations[PATTERN_LEN] = {
  // Intro (ms)
   462,  462,  924,  462,
   462,  924,  462,  924,
  // Intro repeat
   462,  462,  924,  462,
   462,  924,  462,  924,

  // Chorus (ms)
   231,  231,  462,  462,
   231,  231,  462,  231,
   231,  231,  462,  924
};
int GalagaEnemyShotMelody[] = {
    NOTE_G5,   //  784 Hz
    NOTE_GS5,  //  831 Hz
    NOTE_A5,   //  880 Hz
    NOTE_AS5,  //  932 Hz
    NOTE_B5,   //  988 Hz
    NOTE_C6,   // 1047 Hz
    NOTE_CS6,  // 1109 Hz
    NOTE_D6    // 1175 Hz
};

// Duraciones muy cortas para cada paso (ms)
int GalagaEnemyShotDurations[] = {
    20, 20, 20, 20, 20, 20, 20, 20
};
int GalagaExplosionMelody[] = {
    NOTE_B6,  // pico inicial
    NOTE_A6,
    NOTE_G6,
    NOTE_F6,
    NOTE_E6,
    NOTE_C6,
    NOTE_A5,
    NOTE_F5,
    NOTE_D5,
    NOTE_C5,
    0         // silencio al final
};

// Duraciones (ms) para cada paso
int GalagaExplosionDurations[] = {
    20, 20, 20, 20, 20,
    20, 20, 20, 20, 20,
    200       // mantener silencio
};
int LinganguliMelody[] = {
    NOTE_G4, NOTE_G4, NOTE_G4, NOTE_G4,   // “Ging gang goolie goolie”
    NOTE_C5, NOTE_A4, NOTE_C5, 0,         // “goolie goolie watcha” + silencio
    NOTE_G4, NOTE_G4, NOTE_G4, NOTE_G4,   // repetición “Ging gang goolie goolie”
    NOTE_C5, NOTE_A4, NOTE_C5, 0          // “goolie goolie watcha” + silencio
};

// Duraciones en ms (negra = 600 ms; corchea = 300 ms)
// ajustadas para que la frase encaje en 2/4 a 100 BPM
int LinganguliDurations[] = {
    600, 300, 300, 600,   // compás 1: negra, corchea, corchea, negra
    600, 600, 1200, 600,  // compás 2: negra, negra, blanca, negra (silencio)
    600, 300, 300, 600,   // compás 3: como compás 1
    600, 600, 1200, 600   // compás 4: como compás 2
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
int presForFrecuency(int frecuency);
void playTone(int * tone, int * duration, int * pause, int size);
void noTone(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void transmit_uart(char * string){
	uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart1, (uint8_t*)string, len, 200);
}
int presForFrecuency(int frecuency){
	if (frecuency == 0)
		return 0;
	return ((TIM_FREQ/ (ARR * frecuency)) - 1);
}
void playTone(int * tone, int * duration, int * pause, int size){
  for (int i = 0; i < size; i++){
    // 1) si te han pedido parar, apaga y sal
    if (stop_request) {
      noTone();
      return;
    }

    // 2) configura el PWM o silencio
    if (tone[i] == 0) {
      noTone();
    } else {
      __HAL_TIM_SET_PRESCALER(&htim1, presForFrecuency(tone[i]));
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    }

    // 3) toque de la nota con busy-wait, saliendo si llegan más paradas
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < (uint32_t)duration[i]) {
      if (stop_request) {
        noTone();
        return;
      }
    }

    // 4) cortar la nota
    noTone();

    // 5) pausa entre notas (si aplica), también vigilando stop_request
    if (pause) {
      uint32_t pd = (uint32_t)(pause[i] - duration[i]);
      start = HAL_GetTick();
      while ((HAL_GetTick() - start) < pd) {
        if (stop_request) {
          noTone();
          return;
        }
      }
    }
  }
  noTone();
}



/* USER CODE BEGIN 0 */
void playShotBlocking(void) {
    // configura frecuencia de disparo
    __HAL_TIM_SET_PRESCALER(&htim1, presForFrecuency(shotFreq));
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < shotDur) { /* busy-wait */ }
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}
void playShotSequence(void) {
    // pausar Marcha Imperial si estaba en curso
    if (impoPlaying) {
        uint32_t elapsed = HAL_GetTick() - impoStart;
        uint32_t fullDur = ImperialMarchDurations[impoIdx];
        impoRemain = (elapsed < fullDur) ? (fullDur - elapsed) : 0;
        noTone();
        impoPlaying = false;
    }
    // sonar secuencia completa de disparo
    playTone(
        GalagaShotMelody,
        GalagaShotDurations,
        NULL,
        sizeof(GalagaShotMelody)/sizeof(int)
    );
}
void noTone(void){
    // Deja el canal inactivo
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3, &rx3, 1);
  HAL_UART_Receive_IT(&huart1, &rx1, 1);

HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (shotRequested) {
	         shotRequested = false;
	         stop_request  = false;
	         playShotSequence();  // tu función que pausa/reanuda Marcha Imperial
	     }

	     // — disparo enemigo —
	     if (enemyShotRequested) {
	         enemyShotRequested = false;
	         stop_request       = false;
	         // pausar igualmente la marcha si toca
	         if (impoPlaying) {
	           noTone();
	           impoPlaying = false;
	         }
	         playTone(
	           GalagaEnemyShotMelody,
	           GalagaEnemyShotDurations,
	           NULL,
	           sizeof(GalagaEnemyShotMelody)/sizeof(int)
	         );
	     }

	     // — explosión —
	     if (explosionRequested) {
	         explosionRequested = false;
	         stop_request       = false;
	         if (impoPlaying) {
	           noTone();
	           impoPlaying = false;
	         }
	         playTone(
	           GalagaExplosionMelody,
	           GalagaExplosionDurations,
	           NULL,
	           sizeof(GalagaExplosionMelody)/sizeof(int)
	         );
	     }

	 //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	// playTone(Feinmelody, Feindurations, NULL, (sizeof(Feinmelody)/sizeof(Feinmelody[0])));
	 //noTone();
	  // -- BUCLE Feinmelody --
	             // siempre aborta cualquier bucle activo
	  if (cmdNumReady) {
	             loopFein     = false;
	             loopImperial = false;
	             stop_request = false;
	              switch (cmdNum) {
	              case '0':              // NUEVO: comando de parada
	                          stop_request = true;
	                          noTone();          // apaga PWM al instante
	                          break;
	                  case '3':
	                	  stop_request = false;
	                      playTone(GalagaShotMelody, GalagaShotDurations, NULL,
	                               sizeof(GalagaShotMelody)/sizeof(int));
	                      break;
	                  /*case '2':
	                      playTone(Feinmelody, Feindurations, NULL,
	                               sizeof(Feinmelody)/sizeof(int));
	                      break;
	                  case '1':
	                      playTone(GalagaEnemyShotMelody, GalagaEnemyShotDurations, NULL,
	                               sizeof(GalagaEnemyShotMelody)/sizeof(int));
	                      break;
	                  case '4':
	                      playTone(GalagaExplosionMelody, GalagaExplosionDurations, NULL,
	                               sizeof(GalagaExplosionMelody)/sizeof(int));
	                      break;
	                  case '5':
	                      playTone(LinganguliMelody, LinganguliDurations, NULL,
	                               sizeof(LinganguliMelody)/sizeof(int));
	                      break;*/
	                  case '6':
	                	  stop_request = false;
	                  	                      playTone(GalagaShotMelody, GalagaShotDurations, NULL,
	                  	                               sizeof(GalagaShotMelody)/sizeof(int));
	                  	                      break;
	                  default:
	                      break;
	              }
	              noTone();
	              cmdNumReady = false;
	          }
	  if (cmdLetReady) {
	          switch (cmdLet) {
	              // Feinmelody: 'p' start, 'r' stop
	              case 'p':
	                  loopFein     = true;
	                  stop_request = false;
	                  break;
	              case 'r':
	                  loopFein     = false;
	                  stop_request = true;
	                  break;

	              // Marcha Imperial: 'b' start, 't' stop
	              case 'b':
	                  loopImperial = true;
	                  stop_request = false;
	                  break;
	              case 't':
	                  loopImperial = false;
	                  stop_request = true;
	                  break;
	          }
	          cmdLetReady = false;
	      }

	      // 3) Si alguno de los bucles está activo, lo ejecutas
	      if (loopFein && !stop_request) {
	          playTone(Feinmelody,
	                   Feindurations,
	                   NULL,
	                   sizeof(Feinmelody)/sizeof(int));
	          // cuando termine la primera pasada, si sigue loopFein lo repetirá
	      }

	      if (loopImperial) {
	                  stop_request = true;
	                  uint32_t now = HAL_GetTick();
	                  if (!impoPlaying) {
	                      // iniciar o reanudar nota
	                      int note = ImperialMarchMelody[impoIdx];
	                      uint32_t dur = impoRemain ? impoRemain : ImperialMarchDurations[impoIdx];
	                      impoStart   = now;
	                      impoPlaying = true;
	                      impoRemain  = 0;
	                      if (note == 0) {
	                          noTone();
	                      } else {
	                          __HAL_TIM_SET_PRESCALER(&htim1, presForFrecuency(note));
	                          HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	                      }
	                  } else {
	                      // verificar fin de nota
	                      uint32_t elapsed = now - impoStart;
	                      uint32_t fullDur = ImperialMarchDurations[impoIdx];
	                      if (elapsed >= fullDur) {
	                          noTone();
	                          impoPlaying = false;
	                          impoIdx = (impoIdx + 1) % (sizeof(ImperialMarchMelody)/sizeof(int));
	                      }
	                  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  HAL_UART_Receive_IT(&huart1, &rx1, 1);
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_UART_Receive_IT(&huart2, &rx2, 1);
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  HAL_UART_Receive_IT(&huart3, &rx3, 1);
   HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        // — ECO LOCAL (UART1) + MIRROR DEBUG(entre 1→2)
        HAL_UART_Transmit(&huart1, &rx1, 1, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart3, &rx1, 1, HAL_MAX_DELAY);

        // — LÓGICA DE DISPARO O COMANDO —

        // — REHABILITAR INTERRUPCIÓN UART1 —
        HAL_UART_Receive_IT(&huart1, &rx1, 1);
    }
    else if (huart->Instance == USART3) {
        // eco local en puerto 3
        HAL_UART_Transmit(&huart3, &rx3, 1, HAL_MAX_DELAY);

        if (rx3 == '0') {
        	 loopImperial = false;
        	            loopFein     = false;
        	            stop_request = true;
        	            impoPlaying  = false;
        	            noTone();
        }
        else if (rx3 == '7') {
            enemyShotRequested = true;
        }
        else if (rx3 == '8') {
            explosionRequested = true;
        }
        else if (rx3 == '9') {
            shotRequested = true;
        }
        else {
            cmdLet     = rx3;
            cmdLetReady= true;
        }
        // rearmar interrupción en 3
        HAL_UART_Receive_IT(&huart3, &rx3, 1);
    }

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
