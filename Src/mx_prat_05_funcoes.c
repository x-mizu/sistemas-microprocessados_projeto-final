/* ----------------------------------------------------------------------------
 UFABC - Disciplina Sistemas Microprocessados - SuP - 2019.09
 Programa: pPrat_05 - arquivo de funções: prat_05_funcoes.c
 Autor:     Joao Ranhel
 Descricao: contém as funções para criar o programa CONVERSOR ADC prat_05
 Usa:  arquivo 'prat_05_funcoes.h" que contém os #defines e protótipos
       de todas as funções descritas aqui
/  --------------------------------------------------------------------------*/
#include "main.h"
#include "mx_prat_05_funcoes.h"

/* ------ FUNCAO que converte um inteiro hexadecimal em 7 segmentos --------
argmento:  NumHex(valor hexa) + kte TIPO_DISPLAY="0" anodo comum
Author: Joao Ranhel
Ordem dos bits no registrador de deslocamento:
dp g f e d c b a 0 0 0 0 0 0 0 0       (fara' um OR no retorno)
OBS: esta rotina nao liga  o DP...
 ----------------------------------------------------------------------------*/
int16_t conv_7_seg(int NumHex)
{
  int16_t sseg = 0xFF00;
  switch(NumHex)                     // valores default p/ ANODO comum
  {
    case 0: {sseg = 0xC000; break;}  // retorna val p/ 0
    case 1: {sseg = 0xF900; break;}  // retorna val p/ 1
    case 2: {sseg = 0xA400; break;}  // retorna val p/ 2
    case 3: {sseg = 0xB000; break;}  // retorna val p/ 3
    case 4: {sseg = 0x9900; break;}  // retorna val p/ 4
    case 5: {sseg = 0x9200; break;}  // retorna val p/ 5
    case 6: {sseg = 0x8200; break;}  // retorna val p/ 6
    case 7: {sseg = 0xF800; break;}  // retorna val p/ 7
    case 8: {sseg = 0x8000; break;}  // retorna val p/ 8
    case 9: {sseg = 0x9000; break;}  // retorna val p/ 9
    case 10: {sseg = 0x8800; break;} // retorna val p/ A
    case 11: {sseg = 0x8300; break;} // retorna val p/ B
    case 12: {sseg = 0xC600; break;} // retorna val p/ C
    case 13: {sseg = 0xA100; break;} // retorna val p/ D
    case 14: {sseg = 0x8600; break;} // retorna val p/ E
    case 15: {sseg = 0x8E00; break;} // retorna val p/ F
    case 16: {sseg = 0xFF00; break;} // default = tudo desligado
    default: {sseg = 0xBF00; break;} // ERRO retorna "-" (so' g ligado)
  }
  if (TIPO_DISPLAY == 0)             // ANODO COMUM sai como a tabela
    return sseg;
  else                               // CATODO inverte bits (bitwise)
    return ~sseg;
}

// FUNCAO que serializa os dados de 'ser_data' o 74HC595
void serializar(int ser_data)
{
  int stts = 15;                 // envia bit MSB 1o. = dp na placa
  do
  {
    if ((ser_data >> stts) & 1)  // se ser_data desloc >> ssts ='1'
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   // SDATA=1
    } else {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // SDATA=0
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);      // SCK=1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);    // SCK=0
    stts--;
  } while (stts>=0);
	// depois de serializar tudo, tem que gerar RCK
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);      // RCK=1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);    // RCK=0
}

// esta função zera os pinos ao inicializar a placa
void reset_pin_GPIOs (void)
{
  // garantir que pinos serial comecam com zero
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);     // SDATA=0
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);      // SCK=0
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);      // RCK=0
}

// -- (parte b) -- final das demais funções do programa
