/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "mx_prat_05_funcoes.h" // header do arqv das funcoes do programa
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DT_D 100					// delay dos leds
#define reset_time 3000				// tempo necessario para resetar o programa ao apertar PA3
#define min_delay 50				// tempo de delay para incrementar os minutos
#define tempoRespostaMAX 500 // tempo maximo de resposta da UART
#define tempoRequiscoes 200 // tempo entre requisicoes da UART
#define requisicoesCheckSumIncorretoMAX 3 // numero maximo de requisoces incorretas que chegam
#define DT_VARRE 5 // inc varredura a cada 5 ms (~200 Hz)
#define DIGITO_APAGADO 0x10 // kte valor p/ apagar um dígito no display
#define DT_UPDATE 200 // tempo delay para update do valor
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint8_t buffOut[4] = { 0, 0, 0, 0 }; // buffer de saida
volatile uint8_t buffIn[4] = { 0, 0, 0, 0 }; // buffer de entrada
volatile uint16_t valorConversorRecebido; // valor recebido pela UART
volatile uint16_t val_adc = 0; // valor do conversor
uint8_t r = (uint8_t) 0x72; // byte de requisicao
volatile uint8_t requisicaoAtual = 1; // numero atual da requisicao UART
volatile bool respostaRecebida = false; // flag para indicar que recebeu resposta UART
volatile bool requisicaoEnviada = false; // flag para indicar que enviou requisicao UART
volatile bool requisicaoTimeOutEnviada = false; // flgar para indicar que enviou a ultima requisicao antes do TIMEOUT UART

// variaveis que serao mostradas no display
volatile int mil = 0, // ini decimo
		cen = 0, // ini unidade
		dez = 0, // ini dezena
		uni = 0; // ini unidade

// enum de status da UART
volatile static enum StatusUart {
	TransmitirReq, Idle, ErroCheckSum, ErroConexao
} statusUart = Idle;

// enum de status do programa
volatile static enum StatusPrograma {
	Relogio,
	EditarRelogioHora,
	EditarRelogioMinuto,
	Conversor,
	ConexaoUART,
	Reset
} statusPrograma = Relogio;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void TransmitirRequisicaoUART(void); // metodo para transmitir uma requisicao pela UART
void TransmitirDadosUART(void); // metodo para transmitir dados pela UART

// funcoes do arquivo stm32f1xx_it.c
int get_modo_oper_programa(void); // busca modo de operacao do programa
void set_modo_oper_conversor(int); // seta modo_oper (no stm32f1xx_it.c)
int get_modo_oper_conversor(void); // obtém modo_oper (stm32f1xx_it.c)
void reset_modo_oper_programa(void); // reset de modo de operacao do programa
int get_modo_edicao_hora(void); // busca modo de edicao da hora
void reset_modo_edicao_hora(void); // reset do modo de dedicao da hora

// funcoes do arquivo prat_05_funcoes.c
void reset_pin_GPIOs(void); // reset pinos da SPI
void serializar(int ser_data); // prot fn serializa dados p/ 74HC595
int16_t conv_7_seg(int NumHex); // prot fn conv valor --> 7-seg
void ConverteValorDoConversor(uint16_t val);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	int16_t val7seg = 0x00FF, // inicia 7-seg com 0xF (tudo apagado)
			serial_data = 0x01FF; // dado a serializar (dig | val7seg)
	uint32_t tIN_varre = 0; // registra tempo última varredura
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
	MX_ADC1_Init();
	MX_USART1_UART_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */
	// inicializa a SPI (pinos 6,9,10 da GPIOB)
	reset_pin_GPIOs();
	// var de estado que controla a varredura (qual display é mostrado)
	static enum {
		DIG_UNI, DIG_DEC, DIG_CENS, DIG_MILS
	} sttVARRE = DIG_UNI;

	HAL_UART_Receive_IT(&huart1, buffIn, sizeof(buffIn)); // inicializa buffer de entrada

	HAL_GPIO_WritePin(GPIOB,
	GPIO_PIN_15 | GPIO_PIN_14 | GPIO_PIN_13 | GPIO_PIN_12 | GPIO_PIN_5,
			GPIO_PIN_SET); // desliga os leds

	static enum {
		INI_D3, LIG_D3, DSLG_D3
	} sttD3 = INI_D3; // var estados de D3
	static enum {
		INI_D4, LIG_D4, DSLG_D4
	} sttD4 = INI_D4; // var estados de D4
	static enum {
		INI_D, LIG_D, DSLG_D
	} sttD = INI_D; // var estados de D
	static enum {
		UNI, DEZ
	} sttPonto = DEZ; // var ponto display

	// para controlar vars tempos de entrada na rotina ON/OFF de cada LED
	uint32_t tin_D3 = 0, tin_D4 = 0, tin_D = 0;

	// para controlar as vezes que uma requisicao sera feita
	uint32_t tin_UART = 0, timeOut_UART = 0;

	// variaveis para contar hora e minuto
	uint8_t MINuni = 0, MINdec = 0, HRuni = 0, HRdec = 0;

	// variavel para controlar quandos sera incrementado o minuto
	uint32_t tin_min = 0;

	// variavel para controlar quando sera feito o reset do sistema
	uint32_t reset_A3 = 0;

	// variavel para verificar quando o borao PA3 esta sendo pressionado
	bool PA3_pressionado = false;

	// variavel para controlar quando sera feita a conversao AD
	uint32_t tin_conversor = 0;

	// variavel para armazenar o modo de operacao do programa
	int modoDeOperacao = 0;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// Botao PA 3 -----------------------------------------------------------
		// atualiza status do programa de acordo com o modo de operacao EditarRelogio, Conversor, ConexaoUART, Reset
		// Verifica se o botao PA3 esta apertado por 3 segundos
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0) {
			if (PA3_pressionado == false) { // se PA3 pressionado, salva o tick atual
				reset_A3 = HAL_GetTick();
				PA3_pressionado = true;
			}
			if ((HAL_GetTick() - reset_A3) >= reset_time) { // se passou 3 segundos depois do tick inicial
				modoDeOperacao = 5; // muda modo de operacao para RESET
			}
		} else { // se PA3 nao esta pressionado
			PA3_pressionado = false;
			modoDeOperacao = get_modo_oper_programa(); // busca modo de operacao
		}

		// switch para definir o modo de operacao do programa
		switch (modoDeOperacao) {
		case 0:
			statusPrograma = Relogio;
			break;
		case 1:
			statusPrograma = EditarRelogioHora;
			break;
		case 2:
			statusPrograma = EditarRelogioMinuto;
			break;
		case 3:
			statusPrograma = Conversor;
			break;
		case 4:
			statusPrograma = ConexaoUART;
			break;
		case 5:
			statusPrograma = Reset;
			break;
		}

		// --------------------------------------------------------------------------
		// Contador do relogio ------------------------------------------------
		// apenas conta quando nao esta no modo de edicao
		if ((HAL_GetTick() - tin_min) >= min_delay
				&& statusPrograma != EditarRelogioHora
				&& statusPrograma != EditarRelogioMinuto) { //relógio incrementa minutos naturalmente
			tin_min = HAL_GetTick();

			if (MINuni == 9) {
				MINuni = 0;
				if (MINdec == 5) {
					MINdec = 0;
					if (HRuni == 3 && HRdec == 2) {
						HRuni = 0;
						HRdec = 0;
					} else if (HRuni == 9) {
						HRuni = 0;
						HRdec++;
					} else
						HRuni++;

				} else
					MINdec++;
			} else
				MINuni++;
		}

		// --------------------------------------------------------------------------
		// Mostra Relogio no Display---------------------------------------------
		if (statusPrograma == Relogio) {
			// atribui as variaveis que serao mostradas no display
			uni = HRdec;
			dez = HRuni;
			cen = MINdec;
			mil = MINuni;
			sttPonto = DEZ; // coloca o ponto na segunda casa do display
			reset_modo_edicao_hora(); // coloca modo de dedicao de hora para 0
			HAL_GPIO_WritePin(GPIOB,
			GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
					GPIO_PIN_SET); // desl LED
		}
// --------------------------------------------------------------------------
// Editar Relogio--------------------------------------------------------
		if (statusPrograma == EditarRelogioHora) {
			HAL_GPIO_WritePin(GPIOB,
			GPIO_PIN_12 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET); // desl o LED
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // liga o LED 3

			// atribui as variaveis que serao mostradas no display
			uni = HRdec;
			dez = HRuni;
			cen = MINdec;
			mil = MINuni;
			sttPonto = DEZ; // coloca o ponto na segunda casa do display

			if (get_modo_edicao_hora() == 1) {		// PA1 pressionado
				if (HRuni == 3 && HRdec == 2) {	// caso esteja em 23h mudar para 00h
					HRuni = 0;
					HRdec = 0;
				} else if (HRuni == 9) {
					HRuni = 0;
					HRdec++;
				} else
					HRuni++; 	// soma as horas
			}
			reset_modo_edicao_hora(); // volta o modo de edicao hora para 0 (este modo indica quando sera incrementado a unidade)
		}

		// mesma logica que editar a hora
		if (statusPrograma == EditarRelogioMinuto) {
			HAL_GPIO_WritePin(GPIOB,
			GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET); // desl o LED
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // liga o LED 4

			uni = HRdec;
			dez = HRuni;
			cen = MINdec;
			mil = MINuni;
			sttPonto = DEZ;

			if (get_modo_edicao_hora() == 1) {		// PA1 pressionado
				if (MINuni == 9) {
					MINuni = 0;
					if (MINdec == 5)
						MINdec = 0;
					else
						MINdec++;
				} else
					MINuni++;	// soma os minutos
			}
			reset_modo_edicao_hora();
		}
// --------------------------------------------------------------------------
// Conversor AD----------------------------------------------------------
		// adaptado do programa feito em aula
		if (statusPrograma == Conversor) {
			sttPonto = UNI;
			// converte para mVs (decimal, p/ 7-seg)
			if ((HAL_GetTick() - tin_conversor) > DT_UPDATE) {
				tin_conversor = HAL_GetTick();
				ConverteValorDoConversor(val_adc);
			}
			HAL_GPIO_WritePin(GPIOB,
			GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_SET); // desl o LED
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // liga o LED 1
		}

		// faz uma conversão ADC
		if ((HAL_GetTick() - tin_conversor) > DT_UPDATE) {
			// dispara por software uma conversão ADC
			HAL_ADC_Start_IT(&hadc1); // dispara ADC p/ conversão por interrupção
		}
		// --------------------------------------------------------------------------

		// Atualiza display com o valor de uni, dez, cen e mil -----------------------
		// nao depende do modo de operacao do programa
		if ((HAL_GetTick() - tIN_varre) > DT_VARRE) // se ++0,1s atualiza o display
		{
			tIN_varre = HAL_GetTick(); // salva tIN p/ prox tempo varredura
			switch (sttVARRE) // teste e escolha de qual DIG vai varrer
			{
			case DIG_MILS: {
				sttVARRE = DIG_CENS; // ajusta p/ prox digito
				serial_data = 0x0008; // display #1
				val7seg = conv_7_seg(mil);
				break;
			}
			case DIG_CENS: {
				sttVARRE = DIG_DEC; // ajusta p/ prox digito
				serial_data = 0x00004; // display #2
				if (sttPonto == UNI) { // caso o ponto seja na unidade (modo conversor)
					if (cen > 0 || dez > 0 || uni > 0) {
						val7seg = conv_7_seg(cen);
					} else {
						val7seg = conv_7_seg(DIGITO_APAGADO);
					}
				} else { // caso o ponto seja na dezena (modo relogio)
					val7seg = conv_7_seg(cen);
				}

				break;
			}
			case DIG_DEC: {
				sttVARRE = DIG_UNI; // ajusta p/ prox digito
				serial_data = 0x0002; // display #3
				if (sttPonto == UNI) {
					if (dez > 0 || uni > 0) {
						val7seg = conv_7_seg(dez);
						val7seg &= 0xFFFF; // desliga ponto decimal
					} else {
						val7seg = conv_7_seg(DIGITO_APAGADO);
					}
				} else {
					val7seg = conv_7_seg(dez);
					val7seg &= 0x7FFF; // liga o ponto decimal
				}

				break;
			}
			case DIG_UNI: {
				sttVARRE = DIG_MILS; // ajusta p/ prox digito
				serial_data = 0x0001; // display #3
				if (sttPonto == UNI) {
					if (uni > 0) {
						val7seg = conv_7_seg(uni);
						val7seg &= 0x7FFF; // liga o ponto decimal

					} else {
						val7seg = conv_7_seg(DIGITO_APAGADO);
					}
				} else {
					val7seg = conv_7_seg(uni);
					val7seg &= 0xFFFF; // desliga ponto decimal
				}

				break;
			}
			} // fim case
			tIN_varre = HAL_GetTick(); // tmp atual em que fez essa varredura
			serial_data |= val7seg; // OR com val7seg = dado a serializar
			serializar(serial_data); // serializa dado p/74HC595 (shift reg)
		} // -- fim da tarefa #3 - varredura do display

		/// Conexao UART -------------------------------------------------------
		if (statusPrograma == ConexaoUART) {
			switch (statusUart) {
			case Idle: // incializa variaveis para iniciar a comunicacao
				tin_UART = HAL_GetTick();
				timeOut_UART = HAL_GetTick();
				respostaRecebida = false;
				requisicaoEnviada = false;
				requisicaoTimeOutEnviada = false;
				statusUart = TransmitirReq;
				break;
			case TransmitirReq: // se o status for de enviar uma requisicao

				if ((HAL_GetTick() - tin_UART) > tempoRequiscoes
						&& requisicaoEnviada == false) { // se for tempo de fazer uma requisicao
					tin_UART = HAL_GetTick();
					timeOut_UART = HAL_GetTick();
					requisicaoEnviada = true;
					TransmitirRequisicaoUART();
				}

				// se deu 500 ms, envia mais uma requisicao
				if ((HAL_GetTick() - timeOut_UART) > tempoRespostaMAX
						&& requisicaoTimeOutEnviada == false) {
					requisicaoTimeOutEnviada = true;
					TransmitirRequisicaoUART();
				}

				// se depois da requisicao de 500ms, ainda nao recebi resposta depois de 500ms
				// erro de conexao
				if ((HAL_GetTick() - timeOut_UART) > 2 * tempoRespostaMAX) {
					statusUart = ErroConexao;
					HAL_GPIO_WritePin(GPIOB,
					GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
							GPIO_PIN_SET); // desl o LED
				}

				// recebi resposta, reseta variaveis de requisicao
				if (respostaRecebida == true) {
					requisicaoTimeOutEnviada = false;
					requisicaoEnviada = false;
					respostaRecebida = false;
					HAL_GPIO_WritePin(GPIOB,
					GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_SET); // desl o LED
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // desl o LED
				}
				if (requisicaoEnviada == true) {
					switch (sttD) {
					case INI_D:              // vai iniciar a máquina de estado
						tin_D = HAL_GetTick(); // tempo inicial que iniciou a tarefa
						sttD = LIG_D;            // prox estado da máquina
						HAL_GPIO_WritePin(GPIOB,
						GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
								GPIO_PIN_SET); // desl o LED
						break;
					case LIG_D:                 // estado para ligar o LED
						if ((HAL_GetTick() - tin_D) > DT_D) {
							tin_D = HAL_GetTick(); // guarda tempo p/ prox mudança estado
							sttD = DSLG_D;    // muda o prox estado da máquina
							HAL_GPIO_WritePin(GPIOB,
									GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14
											| GPIO_PIN_15, GPIO_PIN_RESET); // ligaLED
						}
						break;
					case DSLG_D:                // estado para desligar o LED
						if ((HAL_GetTick() - tin_D) > DT_D) {
							tin_D = HAL_GetTick(); // guarda tempo p/ prox mudança estado
							sttD = LIG_D;     // muda o prox estado da máquina
							HAL_GPIO_WritePin(GPIOB,
									GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14
											| GPIO_PIN_15, GPIO_PIN_SET); // desl LED
						}
						break;
					};
				}

				break;
			case ErroCheckSum: // se o status for de erro de checksum pisca LED 3
				uni = 0;
				dez = 0;
				cen = 2;
				mil = 4;
				switch (sttD3) {
				case INI_D3: // vai iniciar a máquina de estado
					tin_D3 = HAL_GetTick(); // tempo inicial que iniciou a tarefa
					sttD3 = LIG_D3; // prox estado da máquina
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_5,
							GPIO_PIN_SET); // desl o LED
					break;
				case LIG_D3: // estado para ligar o LED
					if ((HAL_GetTick() - tin_D3) > DT_D) // se for hora de ligar o led
					{
						tin_D3 = HAL_GetTick(); // guarda tempo p/ prox mudança estado
						sttD3 = DSLG_D3; // muda o prox estado da máquina
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_5,
								GPIO_PIN_RESET); // ligaLED
					}
					break;
				case DSLG_D3: // estado para desligar o LED
					if ((HAL_GetTick() - tin_D3) > DT_D) // se for hora de desligar o led
					{
						tin_D3 = HAL_GetTick(); // guarda tempo p/ prox mudança estado
						sttD3 = LIG_D3; // muda o prox estado da máquina
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_5,
								GPIO_PIN_SET); // desl LED
					}
					break;
				}
				break;
			case ErroConexao: // se o status for de erro de conexao pisca LED 4
				uni = 0;
				dez = 6;
				cen = 6;
				mil = 6;
				switch (sttD4) {
				case INI_D4: // vai iniciar a máquina de estado
					tin_D4 = HAL_GetTick(); // tempo inicial que iniciou a tarefa
					sttD4 = LIG_D4; // prox estado da máquina
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_5,
							GPIO_PIN_SET); // desl o LED
					break;
				case LIG_D4: // estado para ligar o LED
					if ((HAL_GetTick() - tin_D4) > DT_D) // se for hora de ligar o led
					{
						tin_D4 = HAL_GetTick(); // guarda tempo p/ prox mudança estado
						sttD4 = DSLG_D4; // muda o prox estado da máquina
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_5,
								GPIO_PIN_RESET); // ligaLED
					}
					break;
				case DSLG_D4: // estado para desligar o LED
					if ((HAL_GetTick() - tin_D4) > DT_D) // se for hora de desligar o led
					{
						tin_D4 = HAL_GetTick(); // guarda tempo p/ prox mudança estado
						sttD4 = LIG_D4; // muda o prox estado da máquina
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_5,
								GPIO_PIN_SET); // desl LED
					}
					break;
				}
			}
		} else {
			// se nao esta no modo de conexao, volta ao status idle e desliga o buzzer
			statusUart = Idle;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // desl buzzer
		}

		// Reset ----------------------------------------------------------------
		if (statusPrograma == Reset) {
			statusUart = Idle;
			respostaRecebida = false;
			requisicaoEnviada = false;
			requisicaoTimeOutEnviada = false;
			HRuni = 0, HRdec = 0, MINuni = 0, MINdec = 0;
			reset_modo_oper_programa();

			HAL_GPIO_WritePin(GPIOB,
			GPIO_PIN_15 | GPIO_PIN_14 | GPIO_PIN_13 | GPIO_PIN_12 | GPIO_PIN_5,
					GPIO_PIN_SET); // desliga os leds

			HAL_UART_Receive_IT(&huart1, buffIn, sizeof(buffIn)); // inicializa buffer de entrada
		}

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
	/* EXTI1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	/* EXTI2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	/* EXTI3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	/* USART1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15
					| GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA1 PA2 PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB10 PB12 PB13 PB14
	 PB15 PB5 PB6 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14
			| GPIO_PIN_15 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// callback do envio de dados
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
	}
}

// callback do recebimento de dados
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {

		if (buffIn[0] == r && buffIn[1] == r && buffIn[2] == r
				&& buffIn[3] == r) { // caso seja uma requisição de dados

			TransmitirDadosUART();

		} else { // senao eu estou recebendo um dado
			uint16_t checkSumCalculado = (uint16_t) buffIn[0] + buffIn[1]; // faz a soma da parte alta com a baixa
			uint16_t checkSumRecebido = ((uint16_t) buffIn[2] << 8) | buffIn[3]; // monta o checksum recebido

			respostaRecebida = true; // recebi uma resposta

			if (checkSumCalculado == checkSumRecebido) { // se o checksum for correto
				// salva valor recebido em uma variavel de 16 bits
				valorConversorRecebido = ((uint16_t) buffIn[0] << 8)
						| buffIn[1];
				ConverteValorDoConversor(valorConversorRecebido);
				requisicaoAtual = 1; // reseta variavel de requisicao para 1

			} else { // se o valor recebido nao for o correto, envia outra requisicao
				requisicaoAtual++; // incremente numero da requisicao
				if (requisicaoAtual > requisicoesCheckSumIncorretoMAX) { // se ja foi enviado 3 requisicoes
					statusUart = ErroCheckSum; // atribui status de erro
					requisicaoAtual = 1;
				}
			}

		}

		HAL_UART_Receive_IT(&huart1, buffIn, sizeof(buffIn)); // inicializa buffer de entrada
	}
}

// transmite uma requisicao
void TransmitirRequisicaoUART(void) {
// prepara requisicao
	buffOut[0] = r;
	buffOut[1] = r;
	buffOut[2] = r;
	buffOut[3] = r;

	HAL_UART_Transmit_IT(&huart1, buffOut, sizeof(buffOut)); // envia requisicao
}

// transmite dados
void TransmitirDadosUART(void) {
	buffOut[1] = (uint8_t) 0xFF & val_adc;
	buffOut[0] = (uint8_t) 0xFF & (val_adc >> 8);
	uint16_t checkSum = (uint16_t) buffOut[0] + buffOut[1];

	buffOut[3] = (uint8_t) 0xFF & checkSum;
	buffOut[2] = (uint8_t) 0xFF & (checkSum >> 8);

	HAL_UART_Transmit_IT(&huart1, buffOut, sizeof(buffOut)); // envia requisicao
}

// converte o valor do conversor para milivolts
void ConverteValorDoConversor(uint16_t val) {
// converter o valor em decimais p/ display
	uint32_t miliVolt = val * 3300 / 4095;
	uni = miliVolt / 1000;
	dez = (miliVolt - (uni * 1000)) / 100;
	cen = (miliVolt - (uni * 1000) - (dez * 100)) / 10;
	mil = miliVolt - (uni * 1000) - (dez * 100) - (cen * 10);
}

// fn que atende ao callback da ISR do conversor ADC1
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
		val_adc = HAL_ADC_GetValue(&hadc1); // capta valor adc
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
