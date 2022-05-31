/*
 * pfcv.c
 *
 *  Created on: Mar 4, 2022
 *      Author: HEIN.H
 */
#include "pfcv.h"
#include <stm32f0xx_hal.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define L_OHM 175000
#define KRYPTON_GAS_CONSTANT 0.605
#define VDC_CONSTANT 0.0009
#define PSI_CONSTANT 19.992
#define PSI_CONSTANT_A 0.1266
#define PSI_CONSTANT_B 1.3
#define PSI_CONSTANT_C 1000000
#define PSI_CONSTANT_D 60

int relation_temp, set_point_pfcv;
double mass_flow_values, set_point;
char pfcv_set_point[4];
float set_point_initial, v_dc, psi;
void pfcv_send_set_point(UART_HandleTypeDef *uart3, int temp, uint8_t *tx,
		uint32_t adcvalue) {

	v_dc = ( VDC_CONSTANT * adcvalue) - 0.028;
	psi = (PSI_CONSTANT * v_dc) - PSI_CONSTANT_A;
	relation_temp = sqrt(530 / (32 + 460));
	set_point_initial = (KRYPTON_GAS_CONSTANT * relation_temp * psi
			* PSI_CONSTANT_B * PSI_CONSTANT_C);
	set_point = set_point_initial / ( L_OHM * PSI_CONSTANT_D);
	set_point_pfcv = (set_point * 64000) / 100;
	itoa(set_point_pfcv, pfcv_set_point, 10);
	tx[1] = pfcv_set_point[0];
	tx[2] = pfcv_set_point[1];
	tx[3] = pfcv_set_point[2];
	tx[4] = pfcv_set_point[3];
	HAL_UART_Transmit(uart3, tx, 6, 30);

}

double pfcv_init(UART_HandleTypeDef *uart3, uint8_t *tx, uint8_t *rx,
		uint8_t *mass_control) {

	HAL_UART_Transmit(uart3, tx, PFCV_TX_BUFFER, 100);
	HAL_UART_Receive(uart3, rx, PFCV_RX_BUFFER, 100);
	memcpy(mass_control, &rx[27], 7);
	mass_flow_values = atof(mass_control);
	return mass_flow_values;

}

