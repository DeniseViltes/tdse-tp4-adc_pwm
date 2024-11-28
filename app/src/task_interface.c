/*
 * task_interface.c
 *
 *  Created on: Nov 27, 2024
 *      Author: ---
 */
#include <stdbool.h>
#include <stdint.h>
#include "task_interface.h"

struct{
	bool flag;
	uint16_t promedio;
}valor_pote;


void init_valor_pote(){
	valor_pote.flag = false;
	valor_pote.promedio = 0;

}

void cargar_valor_pote(uint16_t valor){
	if(valor != valor_pote.promedio){
		valor_pote.flag = true;
		valor_pote.promedio = valor;
	}

}

uint16_t get_valor_pote(){
	valor_pote.flag = false;
	return valor_pote.promedio;
}
