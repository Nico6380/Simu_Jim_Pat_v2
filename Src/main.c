/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * Programa para SIMULAR el software de Jim. El equipo cuenta con una placa
  * para controlar el generador, otra para el calculardor de error, y otra para
  * el patrón.
  *
  * Todas las placas cuentan con un interprete que permite procesar las tramas
  * enviadas.
  *
  * Este firmware es particularmente para la placa del PATRON
  *
  * Mensajes propios que puede recibir:
  * MAR: Indica que se pone en marcha y trae los datos de ensayo
  * STP: Indica que se detuvo el ensayo
  * STD: Pide estado de la medición, tiene que devolver kp y los valores medidos
  *
  * Mensajes que puede enviar:
  * ACK
  * ERR
  * Resultados, no llevan indicador de mensaje
  *
  * Lista de errores
  * 0-> Sin errores, se sigue trabajando
  * 1-> Error de comunicaciones interno (con perifericos del CAL)
  * 2-> El mensaje no es correcto o pide valores sin tener un ensayo en ejecución
  *
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ID1 'P'
#define ID2 'A'
#define ID3 'T'

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void Limpia_Recep(void);	//Limpia el vecto donde recibe la trama
void Limpia_Pr(void);		//Limpia el vector que usa para procesar la trama
int  Valida_Cksum(void); 	//Devuelve el codigo del error, si ocurrió
int  Lectura_Trama(void);	//Devuelve el codigo del error, si ocurrió


void Genera_Trama_ACK_ERROR (void); //Genera la trama a transmitir que contiene el ACK
void Genera_Trama_RLT (void);  		//Genera la trama a transmitir que contiene los resultados
void Genera_Valores_RLT (void); 	//Genera los valores que se van a usar para los resultados

int Valida_Entradas (void);	//Mira las entradas y escribe en el display

void Envio_tx (void);	//Transmite el vector tx


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Variables para la recepción e interpretación:

uint8_t v_rx[267];	//Vector de recepción. Lo que llega se guarda aca
uint8_t v_tx[227];	//Vector de transmisión. Lo que quiero enviar, lo armo acá
uint8_t v_pr[267];	//Vector de procesamiento. Cuando termina de recibir, se pasa acá para poder seguir recibiendo


//uint8_t i_rx=0;		//Contador de posiciónes de v_rx
uint8_t i_pr=0;		//Contador de posiciónes de v_pr

uint16_t l_rx=0;	//Longitud de v_rx
uint16_t l_tx=0;	//Longitud de v_tx
uint8_t l_pr=0;		//Longitud de v_pr

uint8_t a_rx[1];	//Variable auxiliar donde se guarda el caracter recibido.
uint8_t f_rx=0;		//Flag de recepción en proceso
uint8_t f_tc=0;		//Flag de trama completa

uint8_t Error_rx=0;	//Guarda el error de la recepción, en 0 -> no hubo error.
uint8_t Error_cksum=0;	//Guarda si hay el error en el cksum.

//Variables generales
uint8_t estado=0;	//Maquina de estados principal

uint16_t l_max=269;	//Es la cantidad maxima de caracteres que puede tener una trama
uint8_t  l_min = 27; 	//Es la cantidad minima de caracteres que puede tener una trama mensaje mas corto

//Variables para la generación de mediciones:
uint8_t v_rlt[80];	//Valores de los resultados aletorios

//Variables determinar Kp y mediciones
uint8_t Val[9][5]; 	//Valores de tensión corriente y angulo en ASCII
uint8_t kp[10]; 	//Constante del patrón
uint8_t rlt_U[15];  //Valores medidos Tension
uint8_t rlt_I[15];  //Valores medidos Corriete
uint8_t rlt_P[15];  	//Valores medidos Angulo
uint8_t f_rlt=0;	//Flag de solicitud de resultados.
uint8_t inst=0;		//Instrucción solicitada
uint8_t f_mar=0;	//Indica ensayo en marcha

//Variables para el envío de respuestas
uint8_t e0=0;	//Caracter del codigo del error
uint8_t e1=0;	//Caracter del codigo del error


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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //Prearo interrupcion para la recepcion, solo sirve para una vez
  HAL_UART_Receive_IT(&huart1, (uint8_t*)a_rx, sizeof(a_rx));

  HAL_TIM_Base_Start_IT(&htim2);

  //Secuencia de arranque con leds en el frente
  HAL_GPIO_WritePin(LED_V_GPIO_Port, LED_V_Pin, 0);
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 0);

  HAL_Delay(1000);
  HAL_GPIO_WritePin(LED_V_GPIO_Port, LED_V_Pin, 1);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 1);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(LED_V_GPIO_Port, LED_V_Pin, 0);
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 0);

  //Apago el led de la palca y en el frente
  HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_UART_Transmit_IT(&huart1, (uint8_t*) &caracter, sizeof(caracter));
	  switch(estado)
	  {
	  case 0:	//espera recepción, valida, y pasa
	  {
		  if(f_tc==1)	//Si se completó una trama
		  {
			  Limpia_Recep();	//Limpio el vector de recepción y reseteo las variables
			  Error_cksum=Valida_Cksum(); //Valido trama analizo checksum

			  //Si el cksum esta ok, valido la trama y sigo, si hay error en cksum ignoro la trama
			  if(Error_cksum==0 && v_pr[8]==ID1 && v_pr[9]==ID2 && v_pr[10]==ID3)
			  {
				  estado=1; //Si es para mi y cksum ok, avanzo
			  }

			  //Si no valida el cksum o no es para mi el mensaje, lo ignoro
			  else
			  {
				  estado=0;
			  }
		  }
	  } //Fin case 0
	  break;

	  case 1: //Envía respuesta al mensaje recibido y ejecuta
	  {
		  //Lee lo que tiene la trama y guarda los valores en las variables correspondientes
		  //Si hay error de contenido lo reporta
		  Error_rx=Lectura_Trama();

		  //Simulo errores solo si o hubo errores
		  if(Error_rx==0) Error_rx=Valida_Entradas();

		  //Si no hay error en la trama
		  if(Error_rx==0)
		  {
			  if (inst==1) //Marcha
			  {
				  //Prendo led testigo de la placa y en el frente
				  HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, 0);
				  HAL_GPIO_WritePin(LED_V_GPIO_Port, LED_V_Pin, 1);
				  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 0);

				  //Envía marcha al generador con los valores recibidos
				  Genera_Trama_ACK_ERROR();
				  Envio_tx();

				  f_mar=1; //Flag para saber si está funcionando
			  }
			  else if (inst==2)	//Parada
			  {
				  //Apago led testigo
				  HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, 1);
				  HAL_GPIO_WritePin(LED_V_GPIO_Port, LED_V_Pin, 0);
				  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 0);
				  //Envía marcha al generador con los valores recibidos
				  Genera_Trama_ACK_ERROR();
				  Envio_tx();
				  f_mar=0; //Flag para saber si está funcionando

			  }
			  else if (inst==3)//Pide estado
			  {
				  //Si hacemos que pida valores es ahora
				  if(f_mar==0)
				  {
					  //Apago led testigo de la placa y en el frente
					  HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, 1);
					  HAL_GPIO_WritePin(LED_V_GPIO_Port, LED_V_Pin, 0);
					  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 1);

					  Error_rx=2;	//Si pide con el equipo apagado manda error 2
					  Genera_Trama_ACK_ERROR();
					  Envio_tx();
				  }
				  else
				  {
					  //Genero los valores medidos y los envío
					  Genera_Valores_RLT();
					  Genera_Trama_RLT();
					  Envio_tx();

				  }


			  }


			  //segun el comando que recibe, transmite ack o resultados.
		  }

		  //Si hay error en la trama
		  else
		  {
			  //Apago led testigo
			  HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, 1);
			  HAL_GPIO_WritePin(LED_V_GPIO_Port, LED_V_Pin, 0);
			  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 1);

			  //Reporta
			  Genera_Trama_ACK_ERROR();
			  Envio_tx();
		  }

		  //Prepara para volver a recibir
		  Limpia_Recep();
		  Limpia_Pr();
		  estado=0;

	  } //Fin case 1
	  break;

	  }//Fin Switch Principal

	  /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 50;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D1_Pin|D2_Pin|D3_Pin|D4_Pin 
                          |LED_V_Pin|LED_R_Pin|HAB_Tx_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BCD_Out_D_Pin|BCD_Out_C_Pin|BCD_Out_B_Pin|BCD_Out_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Test_Pin */
  GPIO_InitStruct.Pin = Test_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Test_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_Pin D2_Pin D3_Pin D4_Pin 
                           LED_V_Pin LED_R_Pin */
  GPIO_InitStruct.Pin = D1_Pin|D2_Pin|D3_Pin|D4_Pin 
                          |LED_V_Pin|LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BCD_Out_D_Pin BCD_Out_C_Pin BCD_Out_B_Pin BCD_Out_A_Pin */
  GPIO_InitStruct.Pin = BCD_Out_D_Pin|BCD_Out_C_Pin|BCD_Out_B_Pin|BCD_Out_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BCD_In_A_Pin BCD_In_B_Pin BCD_In_C_Pin */
  GPIO_InitStruct.Pin = BCD_In_A_Pin|BCD_In_B_Pin|BCD_In_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HAB_Tx_Pin */
  GPIO_InitStruct.Pin = HAB_Tx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HAB_Tx_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t i_rx=0;
	if (f_rx==0) //Si no está recibiendo
	{
		if (a_rx[0]=='B') //Pregunta si escomienzo de trama
		{
			i_rx=0;
			v_rx[i_rx]=a_rx[0];
			f_rx=1;	//Preparo para seguir ricibiendo la trama
		}
	}
	else	//Si ya comenzó a recibir la trama (recibió la B)
	{
		i_rx++;
		v_rx[i_rx]=a_rx[0];

		if (a_rx[0]=='Z') //Caracter de fin de trama
		{
			l_rx = i_rx + 3;
		}

		if (i_rx==l_rx)	//Llegó al final de la trama
		{
			f_rx=0;	//Preparo para recibir la proxima trama

			i_rx=0;

			l_pr=l_rx; //Guardo la ultima posición del vector recibido

			l_rx=l_max; //maxima longirud para comparar la proxima pasada
			l_rx--;

			f_tc=1; //Flag de trama completa
		}

		}

	//Prearo interrupcion para la recepcion, solo sirve para una vez
	  	HAL_UART_Receive_IT(&huart1, (uint8_t*)a_rx, sizeof(a_rx));

}	//FIN UART

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t aux=0;
	static uint8_t i=0;

	//Lectura de llaves, guardo en una variable que no me interesa, solo quiero actualizar el display
	aux=Valida_Entradas();
	aux++;

	//time to live de la recepción
	if (f_rx==1)
	{
		//Si tarda mas de un segundo en completar la recepción
		if(i==200)
		{
			f_rx=0;
		}
		i++;
	}
	else
	{
		i=0;
	}

//		HAL_GPIO_TogglePin(Test_GPIO_Port, Test_Pin);
}	//Fin timer 2

void Limpia_Recep(void)	//Limpia lo relacionado a la recepción y prepara una nueva
{
	uint16_t i=0;
	for (i=0;i<=l_max-1;i++)
	{
		v_pr[i]=v_rx[i]; //Lleno de espaciós vacios
	}
//
//
//	//Limpia vector de recepcion
//	for (i=0;i<=l_max-1;i++)
//	{
//		v_rx[i]='0'; //Lleno de espaciós vacios
//	}

//	//Limpia vector de transmision
//	for (i=0;i<=l_max-1;i++)
//	{
//		v_tx[i]=' '; //Lleno de espaciós vacios
//	}

	a_rx[0]=0;
//	i_rx=0;
	l_rx=l_max;
	l_rx--;
	f_rx=0;
	f_tc=0;

} //Fin Limpia Recep

void Limpia_Pr(void)
{
	uint16_t i=0;

	for (i=0;i<=l_max-1;i++)
	{
		v_pr[i]=' '; //Lleno de espaciós vacios
	}

} //Fin limpia Pr

int Valida_Cksum(void)
{
	//Variables locales
	uint8_t Error=0;
	uint8_t cksum=0;
	uint16_t i=0;

	//Validación del checksum

	for(i=0;i<=l_pr-1;i++)
	{
		cksum+=v_pr[i];
	}

	if(cksum=='B' || cksum=='Z' || cksum==10) cksum++;

	if(cksum==v_pr[l_pr])	//Verifico el cksm
	{
		Error=0;
	}
	else Error=1;	//No cumple cksm

	//Para probar
//	Error=0;

	return (Error);

} //Fin Valida_Trama

int Lectura_Trama (void)
{
	//Variables locales
	uint8_t Error=0;
	uint8_t i=0;
	uint8_t i_rd=0;	//Posición de lectura
	uint8_t i_wr=0;	//Posición de escritura (fila)
	uint8_t v_ax[10];	//vector auxiliar de lectura

	//El vector tiene datos a partir de la posición 13 inclusive

	//Lectura de la instrucción:
	for(i_rd=13;i_rd<=20;i_rd++)
	{
		v_ax[i_wr]=v_pr[i_rd];
		i_wr++;
	}

		 if(v_ax[0]=='M' && v_ax[1]=='A' && v_ax[2]=='R') inst=1;	//MAR Ensayo en marcha el equipo
	else if(v_ax[0]=='S' && v_ax[1]=='T' && v_ax[2]=='P') inst=2;	//STP Fin de ensayo
	else if(v_ax[0]=='S' && v_ax[1]=='T' && v_ax[2]=='D') inst=3;	//STD Pide mediciones
	else Error=2; //Mensaje incorrecto


	//Si se pone en marcha, guarda los parametros del ensayo
	if(inst==1)
	{
		//Guarda los valores en el vector, restado 48 para que no esté en ASCII
		i=0;
		for(i_rd=26;i_rd<=106;i_rd+=10)
			{
				Val[i][0]=v_pr[i_rd] - 48;
				Val[i][1]=v_pr[i_rd+1] - 48;
				Val[i][2]=v_pr[i_rd+2] - 48;
				Val[i][3]=v_pr[i_rd+3] - 48;
				Val[i][4]=v_pr[i_rd+4] - 48;
				i++;
			}
	}

	return (Error);
} //Fin Lectura_Trama

void Genera_Trama_ACK_ERROR (void) //Genera la trama a transmitir que contiene el ACK con el codigo del error (cero es sin error)
{
	uint8_t i=0;
	uint8_t cksum=0;

	static uint8_t r1=0;
	static uint8_t r2=0;
	static uint8_t r3=0;
	//Convierto el error a ascii, como maximo dos digitos tiene:
	e0=Error_rx%10;	//Obtengo unidad
	e1=Error_rx/10; //Obtengo decena

	if(Error_rx==0)
	{
		r1='A';
		r2='C';
		r3='K';
	}
	else
	{
		r1='E';
		r2='R';
		r3='R';
	}

	//Armo el vector

	v_tx[0] ='B';
	v_tx[1] ='|';
	v_tx[2] =' ';
	v_tx[3] =ID1;	//Origen
	v_tx[4] =ID2;
	v_tx[5] =ID3;
	v_tx[6] ='|';
	v_tx[7] =' ';
	v_tx[8] ='P';	//Destino
	v_tx[9] ='C';
	v_tx[10]='S';
	v_tx[11]='|';
	v_tx[12]=' ';
	v_tx[13]=r1;
	v_tx[14]=r2;
	v_tx[15]=r3;
	v_tx[16]='0';
	v_tx[17]='0';
	v_tx[18]='0';
	v_tx[19]= e1 + 48;
	v_tx[20]= e0 + 48;
	v_tx[21]='|';
	v_tx[22]=' ';
	v_tx[23]='Z';
	v_tx[24]='|';
	v_tx[25]=' ';

	cksum=0;


	for(i=0;i<=25;i++)
	{
		cksum+=v_tx[i];
	}

	if(cksum=='B' || cksum=='Z' || cksum==10) cksum++;

	v_tx[26]=cksum;

	// \n
	v_tx[27]=10;
	l_tx=28;	//Longitud para transmitir

} //Fin Genera Trama ACK

void Genera_Trama_RLT (void)
{
	uint8_t i=0;
	uint8_t k=0;
	uint8_t cksum=0;

	//En una funcion aparte, genero resultados aleatorios para todos los puestos. Cada cierto tiempo habilito que se cargen al vector de transmisión.


	//Siempre envía 20 resultados. Si el puesto esta vacío, no manda nada. Tiene que generar el signo
	//Los resultados los carga cuando pasa cierto tiempo, asi no se actualzan cada vez que se piden resultados

	//Armo cabecera del vector
	v_tx[0] ='B';
	v_tx[1] ='|';
	v_tx[2] =' ';
	v_tx[3] =ID1;	//Origen
	v_tx[4] =ID2;
	v_tx[5] =ID3;
	v_tx[6] ='|';
	v_tx[7] =' ';
	v_tx[8] ='P';	//Destino
	v_tx[9] ='C';
	v_tx[10]='S';
	v_tx[11]='|';
	v_tx[12]=' ';

	//Genero kp, siempre la misma para probar
	v_tx[13]='1';
	v_tx[14]='2';
	v_tx[15]='3';
	v_tx[16]='4';
	v_tx[17]='5';
	v_tx[18]='6';
	v_tx[19]='7';
	v_tx[20]='8';
	v_tx[21]='9';
	v_tx[22]='1';

	//Armo el resto del vector:

	//Tensión
	k=0;
	for(i=23;i<=43;i+=10)
	{
		v_tx[i]='|';
		v_tx[i+1]=' ';
		v_tx[i+2]='0';
		v_tx[i+3]='0';
		v_tx[i+4]='0';
		v_tx[i+5]=rlt_U[k];
		k++;
		v_tx[i+6]=rlt_U[k];
		k++;
		v_tx[i+7]=rlt_U[k];
		k++;
		v_tx[i+8]=rlt_U[k];
		k++;
		v_tx[i+9]=rlt_U[k];
		k++;
	}

	//Corriente
	k=0;
	for(i=53;i<=73;i+=10)
	{
		v_tx[i]='|';
		v_tx[i+1]=' ';
		v_tx[i+2]='0';
		v_tx[i+3]='0';
		v_tx[i+4]='0';
		v_tx[i+5]=rlt_I[k];
		k++;
		v_tx[i+6]=rlt_I[k];
		k++;
		v_tx[i+7]=rlt_I[k];
		k++;
		v_tx[i+8]=rlt_I[k];
		k++;
		v_tx[i+9]=rlt_I[k];
		k++;
	}

	//Angulo
	k=0;
	for(i=83;i<=103;i+=10)
	{
		v_tx[i]='|';
		v_tx[i+1]=' ';
		v_tx[i+2]='0';
		v_tx[i+3]='0';
		v_tx[i+4]='0';
		v_tx[i+5]=rlt_P[k];
		k++;
		v_tx[i+6]=rlt_P[k];
		k++;
		v_tx[i+7]=rlt_P[k];
		k++;
		v_tx[i+8]=rlt_P[k];
		k++;
		v_tx[i+9]=rlt_P[k];
		k++;
	}

	v_tx[113]='|';
	v_tx[114]=' ';
	v_tx[115]='Z';
	v_tx[116]='|';
	v_tx[117]=' ';

	cksum=0;

	for(i=0;i<=117;i++)
	{
		cksum+=v_tx[i];
	}

	if(cksum=='B' || cksum=='Z' || cksum==10) cksum++;

	v_tx[118]=cksum;

	v_tx[119]=10;

	l_tx=120;

}//Fin Genera_Trama_RLT

void Genera_Valores_RLT (void) 	//Genera los valores que se van a usar para los resultados
{

	//Tomamos los valores guardados, restamos uno y sumamos aleatoriamente hasta 2, los decimales los ponemos aleatorios

	uint8_t i=0;
	uint8_t k=0;
	uint16_t V_med[9]; //Guarda los valores en numeros

	//Paso a decimal los valores que cambian,  les resto 10 (el eltimo digito siempre son decimales, y les sumo un random de 20
	V_med[0]=Val[0][1]*1000+Val[0][2]*100+Val[0][3]*10+Val[0][4] - 10 + rand() % 20;
	V_med[1]=Val[1][1]*1000+Val[1][2]*100+Val[1][3]*10+Val[1][4] - 10 + rand() % 20;
	V_med[2]=Val[2][1]*1000+Val[2][2]*100+Val[2][3]*10+Val[2][4] - 10 + rand() % 20;
	V_med[3]=Val[3][0]*10000+Val[3][1]*1000+Val[3][2]*100+Val[3][3]*10+Val[3][4] - 10 + rand() % 20;
	V_med[4]=Val[4][0]*10000+Val[4][1]*1000+Val[4][2]*100+Val[4][3]*10+Val[4][4] - 10 + rand() % 20;
	V_med[5]=Val[5][0]*10000+Val[5][1]*1000+Val[5][2]*100+Val[5][3]*10+Val[5][4] - 10 + rand() % 20;
	V_med[6]=Val[6][2]*100+Val[6][3]*10+Val[6][4] - 10 + rand() % 20;
	V_med[7]=Val[7][2]*100+Val[7][3]*10+Val[7][4] - 10 + rand() % 20;
	V_med[8]=Val[8][2]*100+Val[8][3]*10+Val[8][4] - 10 + rand() % 20;

	k=0;
	//Separo, paso a ASCII y armo el vector Tensión
	for(i=0;i<=14;i+=5)
	{
		//Solo cambian los dos ultimos digitos
		rlt_U[i]  = '0';
		rlt_U[i+1]= (V_med[k]/1000)+48;
		rlt_U[i+2]= ((V_med[k]%1000)/100)+48;
		rlt_U[i+3]= ((V_med[k]%100)/10)+48;
		rlt_U[i+4]= (V_med[k]%10)+48;
		k++;
	}

	//Separo, paso a ASCII y armo el vector Corriente
	for(i=0;i<=14;i+=5)
	{
		//Solo cambian los dos ultimos digitos
		rlt_I[i]  =  (V_med[k]/10000)+48;
		rlt_I[i+1]= ((V_med[k]%10000)/1000)+48;
		rlt_I[i+2]= ((V_med[k]%1000)/100)+48;
		rlt_I[i+3]= ((V_med[k]%100)/10)+48;
		rlt_I[i+4]= (V_med[k]%10)+48;
		k++;
	}

	//Separo, paso a ASCII y armo el vector Angulo
	for(i=0;i<=14;i+=5)
	{
		//Solo cambian los dos ultimos digitos
		rlt_P[i]  = '0';					//Completa la trama de o caracteres
		rlt_P[i+1]= Val[k][1]+48;			//Signo, quedó guardado en matriz de valores recibidos Val[]
		rlt_P[i+2]= (V_med[k]/100)+48; 		//Primer digito queda igual
		rlt_P[i+3]= ((V_med[k]%100)/10)+48;
		rlt_P[i+4]= (V_med[k]%10)+48;
		k++;
	}



} //Genera_Valores_RLT


int Valida_Entradas (void)
{
	// En esta version no se distinuen errores de fases.
	//Lee la llave en BCD de 3 digitos
	uint8_t BCD[3];
	uint8_t valor=0;

	BCD[0]= HAL_GPIO_ReadPin(BCD_In_A_GPIO_Port, BCD_In_A_Pin);
	BCD[1]= HAL_GPIO_ReadPin(BCD_In_B_GPIO_Port, BCD_In_B_Pin);
	BCD[2]= HAL_GPIO_ReadPin(BCD_In_C_GPIO_Port, BCD_In_C_Pin);

	//Calcula el valor
	valor = BCD[0]*1 + BCD[1]*2 + BCD[2]*4;

return(valor);

} //Fin valida entradas

void Envio_tx (void)
{
	//Habilito la transmisión
	  HAL_GPIO_WritePin(HAB_Tx_GPIO_Port, HAB_Tx_Pin, 1);
	  HAL_Delay(50);

	  //Envio
	  HAL_UART_Transmit_IT(&huart1, (uint8_t*) &v_tx, l_tx);
	  HAL_Delay(250);

	  //Deshabilito la transmision
	  HAL_GPIO_WritePin(HAB_Tx_GPIO_Port, HAB_Tx_Pin, 0);
}	//Fin Envio_tx


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
