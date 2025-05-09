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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
#include "bitmaps.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "fatfs_sd.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
	/*/
	 *  p - CANCIÓN MENÚ
	 *  0 - SE ACABA LA CANCIÓN
	 *  9 - DISPARO JUGADOR
	 *  8 - BOOM
	 *  7 - DISPARO ABEJA
	 *  b - BOSS
	 *  0 - NO MORE BOSS
	 */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define GRID_ROWS 240
#define GRID_COLS 320

#define COLOR_BACKGROUND 0x0000
#define COLOR_STARS 0xFFFF
#define NUM_STARS 150
#define NUM_ABEJAS 15
#define LASER_MAX 15
#define NO_JUGADORES 2
#define MUNICION_MAX_JUGADORES 2
#define BOOMS_MAX 25
#define PI 3.14159265358979323846
#define CiclosAbejas_CordsG 10

#define MAX_WIDTH 320
static uint8_t line_buf[MAX_WIDTH * 2];
#define DATA_PA  (LCD_D0_Pin | LCD_D2_Pin | LCD_D7_Pin)
#define DATA_PB  (LCD_D3_Pin | LCD_D4_Pin | LCD_D5_Pin | LCD_D6_Pin)
#define DATA_PC  (LCD_D1_Pin)

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t FASE;
	uint16_t ACTIVO;
} BOOMS;

BOOMS EXP[BOOMS_MAX];

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t x_prev;
	uint16_t y_prev;
	uint8_t Vidas;
	uint8_t Invulnerabilidad;
	uint8_t Municion;
} JUGADORES;

JUGADORES Jugador[NO_JUGADORES];

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t x_prev;
	uint16_t y_prev;
	uint8_t ACTIVO;
}MUNICIONJUGADORES;

MUNICIONJUGADORES LaserJugador[MUNICION_MAX_JUGADORES];

typedef struct {
    uint16_t x;
    uint16_t y;
    uint8_t ACTIVO;
} LASERSTRUCT1;

LASERSTRUCT1 LaserBlue[LASER_MAX];

typedef struct {
    uint16_t x;
    uint16_t y;
} Star;

Star starField[NUM_STARS];

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t X_Pasada1;
    uint16_t Y_Pasada1;
    uint16_t X_Pasada2;
    uint16_t Y_Pasada2;
    uint16_t X_Pasada3;
    uint16_t Y_Pasada3;
    uint16_t X_Pasada4;
    uint16_t Y_Pasada4;
    uint16_t X_Pasada5;
    uint16_t Y_Pasada5;
    uint16_t X_Pasada6;
    uint16_t Y_Pasada6;
    uint16_t X_Pasada7;
    uint16_t Y_Pasada7;
    uint16_t X_Pasada8;
    uint16_t Y_Pasada8;
    uint16_t X_Pasada9;
    uint16_t Y_Pasada9;
    uint16_t X_Pasada10;
    uint16_t Y_Pasada10;
    uint8_t AbejaViva;
} NaveAbeja;

uint8_t NUM_ABEJAS_ACTIVAS = 5;
NaveAbeja ABEJAS[NUM_ABEJAS];

uint8_t velocidad_laser_abejas = 7;
double Dificultad = 1;
double LasersMaxActivos = 0;
uint8_t LaserActivos = 0;

volatile uint16_t posx = 0;
volatile uint16_t posy = 0;

uint8_t rx_byte;
uint8_t esc_state = 0;

bool flag_left_J1  = false;
bool flag_right_J1 = false;
bool flag_disparo_J1 = false;

bool flag_left_J2  = false;
bool flag_right_J2 = false;
bool flag_disparo_J2 = false;

double LMI_Lissajous  = 0.0;
double LMS_Lissajous  = 2.0 * PI;
double dt_Lissajous = 0.02;

double T_Abejas = 0;
uint16_t CiclosAbejas = 0;
uint16_t CiclosAbejasPasado = 0;

uint8_t AbejaSelectDisparo = 0;

uint8_t VelocidadMovimientoJugadorse = 10;
uint16_t AlturaJugadores = 210;
uint8_t VelocidadLaserJugador = 10;

uint8_t Parametro1_Curvas = 2;
uint8_t Parametro2_Curvas = 3;

bool desactivar_laser_abejas = false;
bool BossTime = false;

uint8_t RONDAS = 0;
bool Menu = true;

FATFS fs;
FIL   f;
FRESULT fr;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
char buffer[100];

uint8_t SelectorDificultad = 1;
uint8_t SelectorMenu = 0;

uint16_t Y_JEFE = 0;
uint8_t GrosorLaser = 2;

bool JUEGOTERMINADO = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */



void generateStarField(void);
void drawStars(void);
void ClearStars(void);
void drawImageFromSD(const char *fileName, uint16_t x0, uint16_t y0, uint16_t width, uint16_t height);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t Xprueba = 0;

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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

	  LCD_Init();
	  LCD_Clear(0x0000);


	  for (int i = 0; i < NUM_ABEJAS; i++) {
		  ABEJAS[i].x = 0;
		  ABEJAS[i].y = 0;
		  ABEJAS[i].AbejaViva = 0;
	  }
	  ABEJAS[0].AbejaViva = 1;
	  CiclosAbejas = 0;
	  T_Abejas = 0.0;


	  MX_USART2_UART_Init();
	  HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
	  // <— Inicializa USART2 con los parámetros elegidos

	  //Usar huart2 para TX/RX
	  char *msg = "Hola PC desde Nucleo!\r\n";
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	  uint32_t lastTick = HAL_GetTick();
	  uint32_t lastTickBOOMS = HAL_GetTick();
	  uint32_t lastTickEstrellitas = HAL_GetTick();
	  uint32_t lastTickLasers = HAL_GetTick();
	  uint32_t lastTickMenu = HAL_GetTick();
	  uint32_t lastTickAnJefe = HAL_GetTick();
	  uint32_t lastTickWatingLastAnimation = HAL_GetTick();
	  uint32_t lastTickLaserGrande = HAL_GetTick();

	  uint32_t adcValue;
	  HAL_ADC_Start(&hadc1);
	  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	  {
	      adcValue = HAL_ADC_GetValue(&hadc1);
	  } else
	  HAL_ADC_Stop(&hadc1);
	  srand(adcValue);

	  generateStarField();
	  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);


	  for (int p = 0; p < NO_JUGADORES; p++) {
	      Jugador[p].x = 150;
	      Jugador[p].y = AlturaJugadores;    // ¡inicializa aquí!
	      Jugador[p].x_prev = Jugador[p].x;
	      Jugador[p].y_prev = Jugador[p].y;
	      Jugador[p].Vidas = 30;
	  }


	  bool TecnicaDeHitbox(
	      uint16_t ax, uint16_t ay, uint16_t aw, uint16_t ah,
	      uint16_t bx, uint16_t by, uint16_t bw, uint16_t bh)
	  {
	      return (ax + aw > bx) && (ax < bx + bw) &&
	             (ay + ah > by) && (ay < by + bh);
	  }

	  // SDSDSDSSD---

	    fr = f_mount(&fs, "/", 0);
	    if (fr == FR_OK)
	    {
	    	char errMsg[32];
	    	snprintf(errMsg, sizeof(errMsg), "MONTADA\n");
	    	HAL_UART_Transmit(&huart2, (uint8_t*)errMsg, strlen(errMsg), HAL_MAX_DELAY);
	    }
	    else if (fr != FR_OK)
	    {
	    	char errMsg[32];
	    	snprintf(errMsg, sizeof(errMsg), "FALLO EL INICIO\n");
	    	HAL_UART_Transmit(&huart2, (uint8_t*)errMsg, strlen(errMsg), HAL_MAX_DELAY);
	    }

	    HAL_Delay(1500);


	    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	    HAL_SPI_Init(&hspi1);



	    msg = "p";
	    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		drawImageFromSD("FONDO3.raw", 0, 0, 320, 240);
		FillRectFast (275, 115, 15, 15, 0);
		FillRectFast (255, 115, 15, 15, 0);
		FillRectFast (235, 115, 15, 15, 0);

		FillRectFast (255, 200, 20, 20, 0);

	    //SDSDSDSDSD

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
		  uint32_t now = HAL_GetTick();

		  if(JUEGOTERMINADO == true)
		  {
			  HAL_Delay(3000);
			  drawImageFromSD("KANYE2.raw", 0, 0, 320, 240);
			  HAL_Delay(500000);
		  }

		  if(JUEGOTERMINADO == false)
		  {


		  if (BossTime == true)
		  {
			  if (now - lastTick >= 10)
			  {
				  if (now - lastTickWatingLastAnimation >= 5000)
				  {
					  if ((now - lastTickAnJefe >= 5) &&(Y_JEFE < 40))
					  {
						  lastTickAnJefe = now;
						  Y_JEFE = Y_JEFE + 5;
						  drawImageFromSD("JEFE.raw", 4, Y_JEFE, 311, 93);
					  }
				  }

				  if (Jugador[0].Vidas > 0)
				  		  {
				  			  if (flag_left_J1)
				  			  {
				  				  flag_left_J1 = false;
				  				  Jugador[0].x = Jugador[0].x - VelocidadMovimientoJugadorse;
				  				  if (Jugador[0].x < VelocidadMovimientoJugadorse){ Jugador[0].x = VelocidadMovimientoJugadorse;}
				  			  }
				  			  if (flag_right_J1)
				  			  {
				  				  flag_right_J1 = false;
				  				  Jugador[0].x = Jugador[0].x + VelocidadMovimientoJugadorse;
				  				  if (Jugador[0].x > 320 - 25){ Jugador[0].x = 295;}
				  			  }
				  		  }

				  if (Jugador[1].Vidas > 0)
						  {
							  if (flag_left_J2)
							  {
								flag_left_J2 = false;
								  Jugador[1].x = Jugador[1].x - VelocidadMovimientoJugadorse;
								  if (Jugador[0].x < VelocidadMovimientoJugadorse){ Jugador[1].x = VelocidadMovimientoJugadorse;}
							  }
							  if (flag_right_J2)
							  {
								flag_right_J2 = false;
								  Jugador[1].x = Jugador[1].x + VelocidadMovimientoJugadorse;
								  if (Jugador[1].x > 320 - 25){ Jugador[1].x = 295;}
							  }
						  }

				  if (Jugador[0].Vidas > 0)
				 			 {
				 				 if(Jugador[0].x != Jugador[0].x_prev)
				 				 {
				 					 FillRectFast (Jugador[0].x_prev, AlturaJugadores, 20, 20, 0);
				 					 Jugador[0].x_prev = Jugador[0].x;
				 				 }

				 				 LCD_BitmapFast(Jugador[0].x, AlturaJugadores, 20, 20, NavePrincipal1);
				 			 }

				 if (Jugador[1].Vidas > 0)
				 {
					 if(Jugador[1].x != Jugador[1].x_prev)
					 {
						 FillRectFast (Jugador[1].x_prev, AlturaJugadores, 20, 20, 0);
						 Jugador[1].x_prev = Jugador[1].x;
					 }
					 LCD_BitmapFast(Jugador[1].x, AlturaJugadores, 20, 20, NavePrincipal2);
				 }


				 if ((Y_JEFE  == 40) && (now - lastTickLaserGrande >= 60) && (GrosorLaser < 30))
				{
					 lastTickLaserGrande = now;
					 FillRectFast (160 - GrosorLaser/2, 131, GrosorLaser, 100, 0xF800);
					 GrosorLaser = GrosorLaser + 2;

					 if (GrosorLaser == 30)
					 {
						 for (int GROSOR_LASER_ABAJ0 = 0; GROSOR_LASER_ABAJ0 < 320; GROSOR_LASER_ABAJ0++)
						 {
							 FillRectFast (160 - GROSOR_LASER_ABAJ0/2, 200, GROSOR_LASER_ABAJ0, 40, 0xF800);

							 JUEGOTERMINADO = true;
						 }
					 }

				}


			  }


		  }





	  if (Menu == true)
	  {
		  if (now - lastTickMenu >= 100)
		  {

			 lastTickMenu = now;

			 if (SelectorDificultad == 1){FillRectFast (275, 115, 15, 15, 0x07E0);}
			 if (SelectorDificultad == 2){FillRectFast (255, 115, 15, 15, 0x07E0);}
			 if (SelectorDificultad == 3){FillRectFast (235, 115, 15, 15, 0x07E0);}

			  if (flag_right_J1)
			  {
				  flag_right_J1 = false;
				  if (SelectorMenu == 0)
				  {
					  if (SelectorDificultad > 1) {SelectorDificultad--;}
					  if (SelectorDificultad == 1)
					  {
						  FillRectFast (255, 115, 15, 15, 0);
					  }
					  if (SelectorDificultad == 2)
					  {
						  FillRectFast (235, 115, 15, 15, 0);
					  }
				  }
			  }
			  if (flag_left_J1)
			  {
				  flag_left_J1 = false;
				  if (SelectorMenu == 0)
				  {
					  if (SelectorDificultad < 3) {SelectorDificultad++;}
					  if (SelectorDificultad == 2)
					  {
						  FillRectFast (275, 115, 15, 15, 0);
					  }
					  if (SelectorDificultad == 3)
					  {
						  FillRectFast (255, 115, 15, 15, 0);
					  }
				  }

			  }
			  if (flag_disparo_J1)
			  {
				  flag_disparo_J1 = false;
				 if (SelectorMenu == 0){SelectorMenu++;}
				 else{SelectorMenu--;}
				 if (SelectorMenu == 1){FillRectFast (255, 200, 20, 20, 0x07E0);}
				 if (SelectorMenu == 0){FillRectFast (255, 200, 20, 20, 0);}
			  }
			  if (flag_disparo_J2)
			  {
				  flag_disparo_J2 = false;
				 if (SelectorMenu == 1){Menu = false;}

				 if(SelectorDificultad == 1){ LasersMaxActivos = NUM_ABEJAS_ACTIVAS * .5; Jugador[0].Vidas = 50; Jugador[1].Vidas = 50;}
				 if(SelectorDificultad == 2){ LasersMaxActivos = NUM_ABEJAS_ACTIVAS * 1;  Jugador[0].Vidas = 10; Jugador[1].Vidas = 10;}
				 if(SelectorDificultad == 3){ LasersMaxActivos = NUM_ABEJAS_ACTIVAS * 2;  Jugador[0].Vidas = 5; Jugador[1].Vidas = 5;}


				 msg = "0";
				 HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

				 HAL_Delay(50);

				 FillRectFast (0, 0, 320, 240, 0);

			  }




		  }
	  }



if ((Menu == false)&&(BossTime == false))
{


		  if (Jugador[0].Vidas > 0)
		  {
			  if (flag_left_J1)
			  {
				  flag_left_J1 = false;
				  Jugador[0].x = Jugador[0].x - VelocidadMovimientoJugadorse;
				  if (Jugador[0].x < VelocidadMovimientoJugadorse){ Jugador[0].x = VelocidadMovimientoJugadorse;}
			  }
			  if (flag_right_J1)
			  {
				  flag_right_J1 = false;
				  Jugador[0].x = Jugador[0].x + VelocidadMovimientoJugadorse;
				  if (Jugador[0].x > 320 - 25){ Jugador[0].x = 295;}
			  }
			  if (CiclosAbejas > 50)
			  {
				  if (flag_disparo_J1)
				  {
					 flag_disparo_J1 = false;
					 for(int SelectorLaserPosJugador1 = 0; SelectorLaserPosJugador1 < MUNICION_MAX_JUGADORES; SelectorLaserPosJugador1++)
					 {
						 if(LaserJugador[SelectorLaserPosJugador1].ACTIVO == 0)
						 {
							LaserJugador[SelectorLaserPosJugador1].ACTIVO = 1;
							LaserJugador[SelectorLaserPosJugador1].x = Jugador[0].x + 6;
							LaserJugador[SelectorLaserPosJugador1].y = AlturaJugadores - 20;
							char *msg = "9";
							HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							break;
						 }
					 }
				  }
			  }
		  }

		  if (Jugador[1].Vidas > 0)
		  {
			  if (flag_left_J2)
			  {
				  flag_left_J2 = false;
				  Jugador[1].x = Jugador[1].x - VelocidadMovimientoJugadorse;
				  if (Jugador[1].x < VelocidadMovimientoJugadorse){ Jugador[1].x = VelocidadMovimientoJugadorse;}
			  }
			  if (flag_right_J2)
			  {
				  flag_right_J2 = false;
				  Jugador[1].x = Jugador[1].x + VelocidadMovimientoJugadorse;
				  if (Jugador[1].x > 320 - 25){ Jugador[1].x = 295;}
			  }
			  if (CiclosAbejas > 50)
			  {
				  if (flag_disparo_J2)
				  {
					 flag_disparo_J2 = false;
					 for(int SelectorLaserPosJugador2 = 0; SelectorLaserPosJugador2 < MUNICION_MAX_JUGADORES; SelectorLaserPosJugador2++)
					 {
						 if(LaserJugador[SelectorLaserPosJugador2].ACTIVO == 0)
						 {
							LaserJugador[SelectorLaserPosJugador2].ACTIVO = 1;
							LaserJugador[SelectorLaserPosJugador2].x = Jugador[1].x + 6;
							LaserJugador[SelectorLaserPosJugador2].y = AlturaJugadores - 20;
							char *msg = "9";
							HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							break;
						 }
					 }
				  }
			  }
		  }


// ----------------------- ANIMACIONES ----------------------------------



		 if (now - lastTickEstrellitas >= 1500)
		 {
			 lastTickEstrellitas = now;

		 ClearStars();
		 generateStarField();

		 }

		 if (now - lastTick >= 35)
		 {
			 lastTick = now;

			 drawStars();

			 if (Jugador[0].Vidas > 0)
			 {
				 if(Jugador[0].x != Jugador[0].x_prev)
				 {
					 FillRectFast (Jugador[0].x_prev, AlturaJugadores, 20, 20, 0);
					 Jugador[0].x_prev = Jugador[0].x;
				 }

				 LCD_BitmapFast(Jugador[0].x, AlturaJugadores, 20, 20, NavePrincipal1);
			 }

			 if (Jugador[1].Vidas > 0)
			 {
				 if(Jugador[1].x != Jugador[1].x_prev)
				 {
					 FillRectFast (Jugador[1].x_prev, AlturaJugadores, 20, 20, 0);
					 Jugador[1].x_prev = Jugador[1].x;
				 }
				 LCD_BitmapFast(Jugador[1].x, AlturaJugadores, 20, 20, NavePrincipal2);
			 }



			 for(int MovLaserJugador = 0; MovLaserJugador < MUNICION_MAX_JUGADORES; MovLaserJugador++)
			 {
				 if (LaserJugador[MovLaserJugador].ACTIVO == 1)
				 {
					 FillRectFast (LaserJugador[MovLaserJugador].x, LaserJugador[MovLaserJugador].y, 9, 14, 0);

					 if(LaserJugador[MovLaserJugador].y <= VelocidadLaserJugador + 14)
					 {
						 FillRectFast (LaserJugador[MovLaserJugador].x, LaserJugador[MovLaserJugador].y, 9, 14, 0);
						 LaserJugador[MovLaserJugador].ACTIVO = 0;
						 continue;
					 }

					 LaserJugador[MovLaserJugador].y = LaserJugador[MovLaserJugador].y - VelocidadLaserJugador;
					 LCD_BitmapFast(LaserJugador[MovLaserJugador].x, LaserJugador[MovLaserJugador].y, 9, 14, Laser2);
				  }
			  }

			 if (now - lastTickBOOMS >= 100)
			 {
				 lastTickBOOMS = now;
				 for(int BOOMSCONTADOR = 0; BOOMSCONTADOR < BOOMS_MAX; BOOMSCONTADOR++)
				 {
					 if (EXP[BOOMSCONTADOR].ACTIVO == 1)
					 {
						 uint8_t Fase = EXP[BOOMSCONTADOR].FASE;
						 if (Fase == 1)
						 {
							 char *msg = "8";
							 HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
						 }
						 LCD_SpriteFast(EXP[BOOMSCONTADOR].x, EXP[BOOMSCONTADOR].y, 32, 32, BOOM, 5, Fase, false, false, 0);
						 EXP[BOOMSCONTADOR].FASE++;
						 if(EXP[BOOMSCONTADOR].FASE > 4)
						 {
							 EXP[BOOMSCONTADOR].FASE = 0;
							 EXP[BOOMSCONTADOR].ACTIVO = 0;
							 FillRectFast (EXP[BOOMSCONTADOR].x, EXP[BOOMSCONTADOR].y, 32, 32, 0);
							 continue;
						 }

					 }
				 }
			 }

// --------------------------------------------------------------------
// ------------------------------ ABEJAS ------------------------------
// --------------------------------------------------------------------

if (BossTime == false)
{

	//-------------------------NIVELES----------------------
		    if (CiclosAbejas == 200)
		    {
		    	uint8_t ComproAbejas = 0;
		    	for(int AbejasMuertas = 0; AbejasMuertas < NUM_ABEJAS_ACTIVAS; AbejasMuertas++)
		    	{
		    		if(ABEJAS[AbejasMuertas].AbejaViva == 0)
		    		{
		    			ComproAbejas++;
		    		}
		    		else{CiclosAbejas = 200;}
		    	}
		    	if (ComproAbejas == NUM_ABEJAS_ACTIVAS)
		    	{
		    		RONDAS++;
		    		if (RONDAS == 8)
		    		{
		    			for(int LASER_JUGADOR = 0; LASER_JUGADOR < MUNICION_MAX_JUGADORES; LASER_JUGADOR++)
						 {
							 if(LaserJugador[LASER_JUGADOR].ACTIVO == 1)
							 {
								LaserJugador[LASER_JUGADOR].ACTIVO = 0;
								FillRectFast (LaserJugador[LASER_JUGADOR].x, LaserJugador[LASER_JUGADOR].y, 9, 14, 0);
							 }
						 }

		    			BossTime = true;
		    			lastTick = now;
		    			lastTickWatingLastAnimation = now;
		    			char *msg = "b";
		    			HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		    		}
		    		if (RONDAS < 8)
		    		{
						CiclosAbejas = 0;

						ABEJAS[0].AbejaViva = 1;
						if (velocidad_laser_abejas < 18)
						{
							velocidad_laser_abejas = velocidad_laser_abejas + 2;
						}
						if (RONDAS > 4)
						{
							dt_Lissajous = dt_Lissajous - 0.002;
							Parametro1_Curvas = Parametro1_Curvas + 1;
							Parametro2_Curvas = Parametro2_Curvas + 1;
						}
						else
						{
							dt_Lissajous = dt_Lissajous - 0.0015;
							Parametro1_Curvas = Parametro1_Curvas + 2;
							Parametro2_Curvas = Parametro2_Curvas + 2;
						}
		    		}

		    	}
		    }
    //-------------------------NIVELES----------------------


			if (CiclosAbejas < 200) {CiclosAbejas++;}

			T_Abejas = T_Abejas + dt_Lissajous;
			if (T_Abejas > LMS_Lissajous){T_Abejas = 0;}

			for (int DibujoAbejas = 0; DibujoAbejas < NUM_ABEJAS_ACTIVAS; DibujoAbejas++)
			{
				if(CiclosAbejas == DibujoAbejas*10){ABEJAS[DibujoAbejas].AbejaViva = 1;}

				if (ABEJAS[DibujoAbejas].AbejaViva == 1)
				{
					FillRectFast (ABEJAS[DibujoAbejas].x, ABEJAS[DibujoAbejas].y, 32, 32, 0);
				}
					//Antes de Volver a Calcular Los Valores de X Y
					ABEJAS[DibujoAbejas].X_Pasada10 = ABEJAS[DibujoAbejas].X_Pasada9;
					ABEJAS[DibujoAbejas].Y_Pasada10 = ABEJAS[DibujoAbejas].Y_Pasada9;

					ABEJAS[DibujoAbejas].X_Pasada9 = ABEJAS[DibujoAbejas].X_Pasada8;
					ABEJAS[DibujoAbejas].Y_Pasada9 = ABEJAS[DibujoAbejas].Y_Pasada8;

					ABEJAS[DibujoAbejas].X_Pasada8 = ABEJAS[DibujoAbejas].X_Pasada7;
					ABEJAS[DibujoAbejas].Y_Pasada8 = ABEJAS[DibujoAbejas].Y_Pasada7;

					ABEJAS[DibujoAbejas].X_Pasada7 = ABEJAS[DibujoAbejas].X_Pasada6;
					ABEJAS[DibujoAbejas].Y_Pasada7 = ABEJAS[DibujoAbejas].Y_Pasada6;

					ABEJAS[DibujoAbejas].X_Pasada6 = ABEJAS[DibujoAbejas].X_Pasada5;
					ABEJAS[DibujoAbejas].Y_Pasada6 = ABEJAS[DibujoAbejas].Y_Pasada5;

					ABEJAS[DibujoAbejas].X_Pasada5 = ABEJAS[DibujoAbejas].X_Pasada4;
					ABEJAS[DibujoAbejas].Y_Pasada5 = ABEJAS[DibujoAbejas].Y_Pasada4;

					ABEJAS[DibujoAbejas].X_Pasada4 = ABEJAS[DibujoAbejas].X_Pasada3;
					ABEJAS[DibujoAbejas].Y_Pasada4 = ABEJAS[DibujoAbejas].Y_Pasada3;

					ABEJAS[DibujoAbejas].X_Pasada3 = ABEJAS[DibujoAbejas].X_Pasada2;
					ABEJAS[DibujoAbejas].Y_Pasada3 = ABEJAS[DibujoAbejas].Y_Pasada2;

					ABEJAS[DibujoAbejas].X_Pasada2 = ABEJAS[DibujoAbejas].X_Pasada1;
					ABEJAS[DibujoAbejas].Y_Pasada2 = ABEJAS[DibujoAbejas].Y_Pasada1;

					ABEJAS[DibujoAbejas].X_Pasada1 = ABEJAS[DibujoAbejas].x;
					ABEJAS[DibujoAbejas].Y_Pasada1 = ABEJAS[DibujoAbejas].y;


					//Calculando Valores

					if(DibujoAbejas == 0)
					{
						ABEJAS[0].x = (320.0 - 32.0) * sin(Parametro1_Curvas * (T_Abejas)) / 2.0  +  (320.0 - 32.0) / 2.0;
						ABEJAS[0].y = (200.0 - 32.0) * sin(Parametro2_Curvas * (T_Abejas)) / 2.0  +  (240.0 - 120) / 2.0;
					}
					else
					{
						ABEJAS[DibujoAbejas].x = ABEJAS[DibujoAbejas - 1].X_Pasada10;
						ABEJAS[DibujoAbejas].y = ABEJAS[DibujoAbejas - 1].Y_Pasada10;
					}





				//-----------Intento 1 HITBOX ABEJAS ---------------
				if (ABEJAS[DibujoAbejas].AbejaViva == 1)
				{
					for(int HitBoxLaserJugador = 0; HitBoxLaserJugador < MUNICION_MAX_JUGADORES; HitBoxLaserJugador++)
								 {
									 if (LaserJugador[HitBoxLaserJugador].ACTIVO == 1)
									 {
										if(
											(LaserJugador[HitBoxLaserJugador].x > ABEJAS[DibujoAbejas].x) && (LaserJugador[HitBoxLaserJugador].x < ABEJAS[DibujoAbejas].x + 32) &&
											(LaserJugador[HitBoxLaserJugador].y > ABEJAS[DibujoAbejas].y) && (LaserJugador[HitBoxLaserJugador].y < ABEJAS[DibujoAbejas].y + 32)
										  )
										{
											//CHOCARON ALV
										 LaserJugador[HitBoxLaserJugador].ACTIVO = 0;
										 ABEJAS[DibujoAbejas].AbejaViva = 0;
										 FillRectFast (ABEJAS[DibujoAbejas].x, ABEJAS[DibujoAbejas].y, 32, 32, 0);
										 FillRectFast (LaserJugador[HitBoxLaserJugador].x, LaserJugador[HitBoxLaserJugador].y, 9, 14, 0);
										 for(int BOOMSCONTADOR = 0; BOOMSCONTADOR < BOOMS_MAX; BOOMSCONTADOR++)
										 {
											 if (EXP[BOOMSCONTADOR].ACTIVO == 0)
											 {
												EXP[BOOMSCONTADOR].ACTIVO = 1;
												EXP[BOOMSCONTADOR].x = ABEJAS[DibujoAbejas].x;
												EXP[BOOMSCONTADOR].y = ABEJAS[DibujoAbejas].y;
												break;
											 }
										 }
										}
									 }
								  }
				}

				//-----------Intento 1 HITBOX ABEJAS---------------


				//------------Disparos de las Abejas----------------
				if (ABEJAS[DibujoAbejas].AbejaViva == 1)
				{
					int dx_abejas = ABEJAS[DibujoAbejas].x - ABEJAS[DibujoAbejas].X_Pasada1;
					int dy_abejas = ABEJAS[DibujoAbejas].y - ABEJAS[DibujoAbejas].Y_Pasada1;
					int SpriteAbejitas = 0;

					if(abs(dy_abejas) < 5)
					{
						SpriteAbejitas = 1;
					}else{
						SpriteAbejitas = 3;
					}

					bool RotX_Abejas = (dx_abejas > 0);
					bool RotY_Abejas = (dy_abejas > 0);

				LCD_SpriteFast(ABEJAS[DibujoAbejas].x, ABEJAS[DibujoAbejas].y, 32, 32, AbejaAnima8, 8, SpriteAbejitas, RotX_Abejas, RotY_Abejas, 0);
				}
			}


			if ((CiclosAbejas >  NUM_ABEJAS_ACTIVAS*12)&&(now - lastTickLasers >= 300))
						{
							lastTickLasers = now;
							if (LaserActivos < LasersMaxActivos)
							{
								/*
								 uint32_t adcValue = HAL_ADC_GetValue(&hadc1);    // Obtiene el valor (0 - 4095 para 12 bits)
								 srand(adcValue);  // Usa el valor leído del ADC como semilla para rand()
								*/
								bool AvejasVivas = false;
								for (int HayAbejas = 0; HayAbejas < NUM_ABEJAS_ACTIVAS; HayAbejas++)
								{
									if(ABEJAS[HayAbejas].AbejaViva == 1)
									{
										AvejasVivas = true;
										break;
									}
								}


								uint8_t Intentos_While = 0;
								 do
								 {
									 AbejaSelectDisparo = rand() % (NUM_ABEJAS_ACTIVAS);
									 if ((ABEJAS[AbejaSelectDisparo].AbejaViva == 1)&&( LaserBlue[AbejaSelectDisparo].ACTIVO == 0)&&(ABEJAS[AbejaSelectDisparo].y < 120))
									 {
										 LaserActivos++;
										 LaserBlue[AbejaSelectDisparo].ACTIVO = 1;
										 char *msg = "7";
										 HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
										 LaserBlue[AbejaSelectDisparo].x = ABEJAS[AbejaSelectDisparo].x + 32/2 - 7;
										 LaserBlue[AbejaSelectDisparo].y = ABEJAS[AbejaSelectDisparo].y + 16;
										 break;
									 }
									 Intentos_While++;
									 if (Intentos_While > 50){break;}
								 } while(AvejasVivas == true);

							}
						}

						for (int SelectorLaserPos = 0; SelectorLaserPos < LASER_MAX; SelectorLaserPos++)
						{
							if ( LaserBlue[SelectorLaserPos].ACTIVO == 1)
							{
							  FillRectFast (LaserBlue[SelectorLaserPos].x, LaserBlue[SelectorLaserPos].y, 8, 16, 0);
							  LaserBlue[SelectorLaserPos].y  = LaserBlue[SelectorLaserPos].y + velocidad_laser_abejas;
							  LCD_BitmapFast(LaserBlue[SelectorLaserPos].x, LaserBlue[SelectorLaserPos].y, 8, 16, Laser1);
							  if (LaserBlue[SelectorLaserPos].y > 240 - 15)
							  {
								  LaserBlue[SelectorLaserPos].ACTIVO = 0;
								  FillRectFast (LaserBlue[SelectorLaserPos].x, LaserBlue[SelectorLaserPos].y, 8, 16, 0);
								  desactivar_laser_abejas = true;
							  }
							}
						}

			//------------Disparos de las Abejas----------------

			//-----------Intento 1 HITBOX JUGADORES ABEJAS ---------------

			for(int HitBoxLaserAbejas = 0; HitBoxLaserAbejas < LASER_MAX; HitBoxLaserAbejas++)
			{
				if (LaserBlue[HitBoxLaserAbejas].ACTIVO == 1)
				{
					for (int HitBoxJugadores = 0; HitBoxJugadores < NO_JUGADORES; HitBoxJugadores++)
					{
						if( Jugador[HitBoxJugadores].Vidas > 0)
						{
							/*if(
								(LaserBlue[HitBoxLaserAbejas].x > Jugador[HitBoxJugadores].x) && (LaserBlue[HitBoxLaserAbejas].x < Jugador[HitBoxJugadores].x + 20) &&
								(LaserBlue[HitBoxLaserAbejas].y + 16 > Jugador[HitBoxJugadores].y) && (LaserBlue[HitBoxLaserAbejas].y + 16 < Jugador[HitBoxJugadores].y + 20)
							  )*/
							if(TecnicaDeHitbox(
												LaserBlue[HitBoxLaserAbejas].x, LaserBlue[HitBoxLaserAbejas].y, 8, 16,
												Jugador[HitBoxJugadores].x, Jugador[HitBoxJugadores].y, 20, 20))

							{
								 //CHOCARON ALV
								 LaserBlue[HitBoxLaserAbejas].ACTIVO = 0;
								 desactivar_laser_abejas = true;
								 Jugador[HitBoxJugadores].Vidas = Jugador[HitBoxJugadores].Vidas - 1;
								 if (Jugador[HitBoxJugadores].Vidas == 0)
								 {
									 FillRectFast (Jugador[HitBoxJugadores].x, Jugador[HitBoxJugadores].y, 20, 20, 0);
								 }
								 FillRectFast (LaserBlue[HitBoxLaserAbejas].x, LaserBlue[HitBoxLaserAbejas].y, 8, 16, 0);
								 for(int BOOMSCONTADOR = 0; BOOMSCONTADOR < BOOMS_MAX; BOOMSCONTADOR++)
								 {
									 if (EXP[BOOMSCONTADOR].ACTIVO == 0)
									 {
										EXP[BOOMSCONTADOR].ACTIVO = 1;
										EXP[BOOMSCONTADOR].x = Jugador[HitBoxJugadores].x - 6;
										EXP[BOOMSCONTADOR].y = Jugador[HitBoxJugadores].y - 6;
										break;
									 }
								 }
							}
						}
					}
				}
			}

			//-----------Intento 1 HITBOX JUGADORES ABEJAS ---------------

			if (desactivar_laser_abejas){LaserActivos--;desactivar_laser_abejas=false;}
}


			// ------------------------- JEFEEEEEEEEEEEE -------------------------




			// ------------------------- JEFEEEEEEEEEEEE -------------------------
		 }







    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* SPI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, F_CS_Pin|LCD_RST_Pin|LCD_D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_SS_GPIO_Port, SD_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : F_CS_Pin LCD_RST_Pin LCD_D1_Pin */
  GPIO_InitStruct.Pin = F_CS_Pin|LCD_RST_Pin|LCD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_D7_Pin
                           LCD_D0_Pin LCD_D2_Pin */
  GPIO_InitStruct.Pin = LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin
                           LCD_D4_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_SS_Pin */
  GPIO_InitStruct.Pin = SD_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SD_SS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {

    	if (rx_byte == '1') {flag_left_J1  = true;}
    	if (rx_byte == '2') {flag_right_J1 = true;}
    	if (rx_byte == '3') {flag_disparo_J1 = true;}


    	if (rx_byte == '4') {flag_left_J2  = true;}
    	if (rx_byte == '5') {flag_right_J2 = true;}
    	if (rx_byte == '6') {flag_disparo_J2 = true;}
        // volver a armar la recepción

    	/*
    	char *msgMOV = "";

    	if (flag_left_J1)
    	{
    		 msgMOV = "LEFT_J1\r\n";
    		 HAL_UART_Transmit(&huart2, (uint8_t*)msgMOV, strlen(msgMOV), HAL_MAX_DELAY);
    	}
		if (flag_right_J1)
		{
			 msgMOV = "RIGHT_J1\r\n";
			 HAL_UART_Transmit(&huart2, (uint8_t*)msgMOV, strlen(msgMOV), HAL_MAX_DELAY);
		}
		if (flag_left_J2)
		{
			char buf[16];
			snprintf(buf, sizeof(buf), "%d", Jugador[1].x);
			 msgMOV = buf;
			 HAL_UART_Transmit(&huart2, (uint8_t*)msgMOV, strlen(msgMOV), HAL_MAX_DELAY);
		}
		if (flag_right_J2)
		{
			 msgMOV = "RIGHT_J2\r\n";
			 HAL_UART_Transmit(&huart2, (uint8_t*)msgMOV, strlen(msgMOV), HAL_MAX_DELAY);
		}*/
    }

        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

}

void generateStarField(void) {
    // Recorremos el arreglo y asignamos una posición aleatoria a cada estrella
    for (uint16_t i = 0; i < NUM_STARS; i++) {
        starField[i].x = rand() % GRID_COLS;   // Genera x entre 0 y SCREEN_WIDTH - 1
        starField[i].y = rand() % GRID_ROWS;  // Genera y entre 0 y SCREEN_HEIGHT - 1
    }
}

void drawStars(void) {
    for (uint16_t i = 0; i < NUM_STARS; i++)
    {
    	FillRectFast(starField[i].x, starField[i].y, 1, 1,  COLOR_STARS);
    }
}

void ClearStars(void) {
    for (uint16_t i = 0; i < NUM_STARS; i++)
    {
    	FillRectFast(starField[i].x, starField[i].y, 1, 1,  COLOR_BACKGROUND);
    }
}

void drawImageFromSD(const char *fileName,
                     uint16_t x0, uint16_t y0,
                     uint16_t width, uint16_t height)
{
    FIL     file;
    FRESULT fr;
    UINT    br;
    char    msg[64];

    // 1) Abrir el fichero en modo lectura
    fr = f_open(&file, fileName, FA_OPEN_EXISTING | FA_READ);
    if (fr != FR_OK) {
        snprintf(msg, sizeof(msg), "ERR OPEN %s: %d\n", fileName, fr);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    // 2) Leer cada fila de ancho*2 bytes y dibujarla
    for (uint16_t row = 0; row < height; row++) {
        fr = f_read(&file, line_buf, width * 2, &br);
        if (fr != FR_OK || br != width * 2) {
            snprintf(msg, sizeof(msg),
                     "ERR READ row %u: %d br %u\n",
                     row, fr, br);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            break;
        }

        // 3) Dibuja la línea en la pantalla:
        //    ancho=width, alto=1 fila, datos en line_buf

        for (uint32_t i = 0; i < width*2; i += 2) {
                uint8_t tmp       = line_buf[i];
                line_buf[i]       = line_buf[i+1];
                line_buf[i+1]     = tmp;
            }

        LCD_BitmapFast(x0, y0 + row,
                       width, 1,
                       line_buf);
    }

    // 4) Cerrar el fichero
    f_close(&file);
    HAL_UART_Transmit(&huart2,
                      (uint8_t*)"DONE\n",
                      5, HAL_MAX_DELAY);
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
