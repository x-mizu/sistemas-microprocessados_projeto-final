/* -------------- arquivo header para minhas funcoes prat_05 -----------------
Arquivo: prat_05_funcoes.h
Created on: 2019.09
Author: JRanhel
-----------------------------------------------------------------------------*/

#ifndef PRAT_05_FUNCOES_H_
#define PRAT_05_FUNCOES_H_

#define DISP_TIPO 0x0                  // DISP_TIPO=0 ÂNODO comum. Se=1 CATODO

// as constantes usadas nos arquivos com as funcoes.c
#define TIPO_DISPLAY 0                 // tipo de display = Anodo comum

// fn conv um int16_t hexa em 7-seg. Argumentos: NumHex + KTE TIPO_DISPLAY
int16_t conv_7_seg(int NumHex);

// fn serializa dados p/ 74HC595. faz: 16x bit SDATA, SCLK, ao final RCLK
void serializar(int ser_data);         // prot fn serializa dados p/ 74HC595

// esta função zera os pinos ao inicializar a placa
void reset_pin_GPIOs (void);

#endif /* PRAT_05_FUNCOES_H_ */
