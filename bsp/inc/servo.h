/*
 * servo.h
 *
 *  Created on: Oct 22, 2025
 *      Author: thalya.morice-roy-le
 */

#ifndef BSP_INC_SERVO_H_
#define BSP_INC_SERVO_H_

#include "stm32f0xx.h"

#include <stdint.h>

typedef struct {
    uint8_t  id;
    uint16_t goal_pos;
    uint8_t  moving;          // 0 = arrêté, 1 = en mouvement
    uint32_t move_start_ms;   // moment où on a lancé le mouvement
    uint32_t move_duration_ms;// durée estimée du mouvement
} herkulex_servo_t;

void herkulex_servo_init(herkulex_servo_t *s, uint8_t id);
void herkulex_servo_goto(herkulex_servo_t *s, uint16_t position, uint8_t playtime);
void herkulex_servo_update(herkulex_servo_t *s);
uint8_t herkulex_servo_is_done(const herkulex_servo_t *s);

void servo_uart_init();
void uart_send_byte(uint8_t data);
void herkulex_ram_write(uint8_t servo_id, uint8_t address, uint8_t length, uint8_t *data);
void herkulex_ram_read(uint8_t servo_id, uint8_t address, uint8_t length);
void herkulex_led_red(uint8_t servo_id);
void herkulex_torque_on(uint8_t servo_id);
void herkulex_set_position(uint8_t servo_id, uint16_t position, uint32_t playtime);
void herkulex_led_green(uint8_t servo_id);
void herkulex_set_position2(uint8_t  id1, uint16_t position1,uint8_t  id2, uint16_t position2,uint32_t  playtime);
void herkulex_led_cyan(uint8_t servo_id);

#endif /* BSP_INC_SERVO_H_ */
