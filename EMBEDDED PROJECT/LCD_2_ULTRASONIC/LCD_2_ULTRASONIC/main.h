/*
 * main.h
 *
 * Created: 4/30/2025 8:50:42 PM
 *  Author: huawei
 */ 


#ifndef MAIN_H_
#define MAIN_H_

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Definici?n de frecuencia del microcontrolador
#define F_CPU 16000000UL  // 16 MHz

// ========== LCD Definitions ==========
#define LCD_RS      PD0
#define LCD_EN      PD1
#define LCD_D4      PD2
#define LCD_D5      PD3
#define LCD_D6      PD4
#define LCD_D7      PD5
#define LCD_PORT    PORTD
#define LCD_DDR     DDRD
#define LCD_PIN     PIND

// Comandos LCD para modo 4-bit
#define LCD_CLEAR           0x01
#define LCD_HOME            0x02
#define LCD_ENTRY_MODE      0x06  // Incremento cursor, sin desplazamiento
#define LCD_DISPLAY_ON      0x0C  // Display on, cursor off, parpadeo off
#define LCD_FUNCTION_4BIT   0x28  // 4-bit, 2 l?neas, 5x8 dots
#define LCD_FUNCTION_RESET  0x30  // Secuencia de reset
#define LCD_SET_CGRAM       0x40
#define LCD_SET_DDRAM       0x80

// ========== ADC Channels ==========
#define POT1_CHANNEL 1
#define POT2_CHANNEL 0  // (Unused)

// ========== Ultrasonic Sensor ==========
#define US_DDR      DDRA
#define US_PORT     PORTA
#define US_PIN      PINA
#define US1_TRIG_PIN PA3    // PIN_PA3
#define US1_ECHO_PIN PA0    // PIN_PA0
#define US2_TRIG_PIN PA4    // PIN_PA4
#define US2_ECHO_PIN PA2    // PIN_PA2
#define US3_TRIG_PIN PA5    // PIN_PA5 - Nuevo sensor para detectar personas
#define US3_ECHO_PIN PA6    // PIN_PA6 - Nuevo sensor para detectar personas

// Definici?n de pines con n?meros para facilitar su uso en funciones
#define US1_TRIG_PIN_NUM    3
#define US1_ECHO_PIN_NUM    0
#define US2_TRIG_PIN_NUM    4
#define US2_ECHO_PIN_NUM    2
#define US3_TRIG_PIN_NUM    5
#define US3_ECHO_PIN_NUM    6

// ========== Servo Motor ==========
#define SERVO_DDR    DDRD
#define SERVO_PORT   PORTD
#define SERVO_PIN    PD7
#define SERVO_OPEN   2500   // Valor para posici?n abierta (ajustar seg?n tu servo)
#define SERVO_CLOSED 1250   // Valor para posici?n cerrada (ajustar seg?n tu servo)

// ========== Par?metros de la aplicaci?n ==========
#define BIN_DEPTH_CM        50    // Profundidad del contenedor de basura
#define MIN_DISTANCE_CM     5     // Distancia m?nima para considerar 100% lleno
#define PERSON_DISTANCE_CM  30    // Distancia para detectar persona
#define MAX_SENSOR_TIMEOUT  50000 // Timeout para prevenir bloqueos (aumentado)
#define MEASUREMENT_DELAY   500   // Retraso entre mediciones (ms)
#define ULTRASONIC_TIMEOUT  20    // Timeout m?ximo para medici?n (ms) (aumentado)
#define SERVO_OPEN_TIME     3000  // Tiempo que permanece abierto el contenedor (ms)

// ========== Prototipos de funciones LCD ==========
void lcd_init(void);
void lcd_send_command(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_send_byte(uint8_t byte);
void lcd_send_nibble(uint8_t nibble);
void lcd_clear(void);
void lcd_home(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_string(const char *str);
void lcd_create_char(uint8_t location, const uint8_t *charmap);

// ========== Prototipos de funciones ADC ==========
void adc_init(void);
uint16_t adc_read(uint8_t channel);

// ========== Prototipos de funciones de ultrasonido ==========
void ultrasonic_init(void);
uint16_t measure_distance(uint8_t trig_pin, uint8_t echo_pin);
void trig_pulse(uint8_t trig_pin);

// ========== Prototipos de funciones del servo ==========
void servo_init(void);
void servo_position(uint16_t position);
void servo_open(void);
void servo_close(void);

// ========== Funciones de utilidad ==========
void system_init(void);
uint8_t calculate_fill_percentage(uint16_t distance);
void display_bin_status(uint8_t fill1, uint8_t fill2, uint16_t dist1, uint16_t dist2);
void handle_error(uint8_t error_code);

#endif /* MAIN_H_ */