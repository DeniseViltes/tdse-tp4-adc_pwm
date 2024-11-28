/*
 * task_interface.h
 *
 *  Created on: Nov 27, 2024
 *      Author: ---
 */

#ifndef INC_TASK_INTERFACE_H_
#define INC_TASK_INTERFACE_H_


#ifdef __cplusplus
extern "C" {
#endif


//Inicializo en cero y el flag en false
void init_valor_pote();
void cargar_valor_pote(uint16_t valor);
uint16_t get_valor_pote();


#ifdef __cplusplus
}
#endif

#endif /* INC_TASK_INTERFACE_H_ */
