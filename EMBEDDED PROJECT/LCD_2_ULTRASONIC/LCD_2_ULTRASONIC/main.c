/*
 * LCD_2_ULTRASONIC.c
 *
 * Created: 4/30/2025 8:49:56 PM
 * Author : huawei
 */ 
#include "main.h"

// Variable global para contar desbordamientos del temporizador
volatile uint16_t timer_overflow_count = 0;

// Variable para controlar estado del servo
volatile uint8_t servo_state = 0; // 0 = cerrado, 1 = abierto
volatile uint32_t servo_open_timestamp = 0;

// C?digo de errores
enum ErrorCodes {
	ERR_NONE = 0,
	ERR_US_TIMEOUT = 1,
	ERR_LCD_INIT = 2,
	ERR_ADC_READ = 3
};

// Manejador de interrupciones para desbordamiento del Timer1
ISR(TIMER1_OVF_vect) {
	timer_overflow_count++;
}

// ==================== FUNCIONES LCD (MODO 4-BIT) ====================

// Env?a un nibble (4 bits) al LCD
void lcd_send_nibble(uint8_t nibble) {
	// Conservar los bits que no forman parte de los datos
	uint8_t port_state = LCD_PORT & ~((1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7));
	
	// Establece los 4 bits de datos de una manera m?s eficiente
	LCD_PORT = port_state |
	((nibble & 0x01) << LCD_D4) |
	((nibble & 0x02) << (LCD_D5-1)) |
	((nibble & 0x04) << (LCD_D6-2)) |
	((nibble & 0x08) << (LCD_D7-3));
	
	// Pulso de habilitaci?n
	LCD_PORT |= (1 << LCD_EN);
	_delay_us(2);  // Enable pulse debe ser >450ns (ajustado para 16MHz)
	LCD_PORT &= ~(1 << LCD_EN);
	_delay_us(100); // Esperar a que el LCD procese (aumentado para mayor seguridad)
}

// Env?a un byte completo al LCD (dividido en dos nibbles)
void lcd_send_byte(uint8_t byte) {
	// Env?a nibble alto (4 bits superiores)
	lcd_send_nibble(byte >> 4);
	// Env?a nibble bajo (4 bits inferiores)
	lcd_send_nibble(byte & 0x0F);
}

// Env?a un comando al LCD
void lcd_send_command(uint8_t cmd) {
	LCD_PORT &= ~(1 << LCD_RS); // RS = 0 para comandos
	lcd_send_byte(cmd);
	// Esperar a que el comando se ejecute
	if (cmd == LCD_CLEAR || cmd == LCD_HOME)
	_delay_ms(5); // Aumentado para mayor seguridad
	else
	_delay_us(100); // Aumentado para mayor seguridad
}

// Env?a datos al LCD
void lcd_send_data(uint8_t data) {
	LCD_PORT |= (1 << LCD_RS); // RS = 1 para datos
	lcd_send_byte(data);
	_delay_us(100); // Aumentado para mayor seguridad
}

// Inicializa el LCD en modo 4-bit
void lcd_init(void) {
	// Configura pines como salidas
	LCD_DDR |= (1 << LCD_RS) | (1 << LCD_EN) |
	(1 << LCD_D4) | (1 << LCD_D5) |
	(1 << LCD_D6) | (1 << LCD_D7);
	
	// Inicialmente todos los pines en bajo
	LCD_PORT &= ~((1 << LCD_RS) | (1 << LCD_EN) |
	(1 << LCD_D4) | (1 << LCD_D5) |
	(1 << LCD_D6) | (1 << LCD_D7));
	
	// Esperar m?s de 40ms después de encendido
	_delay_ms(100); // Aumentado para mayor seguridad
	
	// Inicializaci?n especial para modo 4-bit
	// Primero aseguramos que RS=0 (modo comando)
	LCD_PORT &= ~(1 << LCD_RS);
	LCD_PORT &= ~(1 << LCD_EN);
	
	// Secuencia de inicializaci?n seg?n datasheet
	// Env?a 0x3 tres veces para asegurar modo 8-bit inicialmente
	lcd_send_nibble(0x03);
	_delay_ms(5);
	lcd_send_nibble(0x03);
	_delay_ms(5); // Aumentado para mayor seguridad
	lcd_send_nibble(0x03);
	_delay_ms(5); // Aumentado para mayor seguridad
	
	// Cambiar a modo 4-bit
	lcd_send_nibble(0x02);
	_delay_ms(5); // Aumentado para mayor seguridad
	
	// Ahora ya estamos en modo 4-bit y podemos usar lcd_send_command()
	lcd_send_command(LCD_FUNCTION_4BIT); // Configurar: 4-bit, 2 l?neas, 5x8 dots
	_delay_ms(1);
	lcd_send_command(LCD_DISPLAY_ON);    // Display on, cursor off, parpadeo off
	_delay_ms(1);
	lcd_send_command(LCD_CLEAR);         // Borrar display
	_delay_ms(5);
	lcd_send_command(LCD_ENTRY_MODE);    // Incremento cursor, sin desplazamiento
	_delay_ms(1);
}

// Borra la pantalla LCD
void lcd_clear(void) {
	lcd_send_command(LCD_CLEAR);
	_delay_ms(5); // Asegurar que la pantalla tenga tiempo de limpiarse
}

// Establece cursor en inicio
void lcd_home(void) {
	lcd_send_command(LCD_HOME);
	_delay_ms(5); // Asegurar que el cursor tenga tiempo de moverse
}

// Establece la posici?n del cursor
void lcd_set_cursor(uint8_t row, uint8_t col) {
	uint8_t address;
	
	// Validar l?mites
	if (row > 1) row = 1;
	if (col > 15) col = 15;
	
	if (row == 0) {
		address = 0x00 + col; // Primera l?nea comienza en 0x00
		} else {
		address = 0x40 + col; // Segunda l?nea comienza en 0x40
	}
	
	lcd_send_command(LCD_SET_DDRAM | address);
	_delay_us(100); // Tiempo para que se mueva el cursor
}

// Env?a una cadena de texto al LCD
void lcd_string(const char *str) {
	uint8_t i = 0;
	
	// Proteger contra cadenas demasiado largas o nulas
	if (!str) return;
	
	while (str[i] != '\0' && i < 16) { // L?mite de 16 caracteres por l?nea
		lcd_send_data(str[i]);
		i++;
	}
}

// Crea un car?cter personalizado
void lcd_create_char(uint8_t location, const uint8_t *charmap) {
	uint8_t i;
	
	// Validar puntero
	if (!charmap) return;
	
	// M?ximo 8 caracteres personalizados (0-7)
	location &= 0x07;
	
	lcd_send_command(LCD_SET_CGRAM | (location << 3));
	
	for (i = 0; i < 8; i++) {
		lcd_send_data(charmap[i]);
	}
	
	// Volver a DDRAM
	lcd_send_command(LCD_SET_DDRAM);
}

// ==================== FUNCIONES ADC ====================

// Inicializa el ADC
void adc_init(void) {
	// Referencia de voltaje: AVCC con capacitor externo en AREF
	ADMUX = (1 << REFS0);
	
	// Habilita ADC, prescaler 128 (frecuencia ADC = 125kHz a 16MHz)
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	// Primera conversi?n dummy (para calibrar)
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC));
}

// Lee un canal ADC
uint16_t adc_read(uint8_t channel) {
	// Validar canal
	if (channel > 7) channel = 7;
	
	// Selecciona canal (0-7)
	ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
	
	// Peque?o retraso para estabilizar el multiplexor
	_delay_us(10);
	
	// Inicia la conversi?n
	ADCSRA |= (1 << ADSC);
	
	// Espera a que termine la conversi?n
	while (ADCSRA & (1 << ADSC));
	
	// Devuelve el resultado
	return ADC;
}

// ==================== FUNCIONES SERVO ====================

// Inicializa el servo motor
void servo_init(void) {
	// Configura pin como salida
	SERVO_DDR |= (1 << SERVO_PIN);
	
	// Inicialmente en posici?n cerrada
	servo_close();
}

// Establece posici?n del servo
void servo_position(uint16_t position) {
	// Limitar el valor a los l?mites seguros para el servo
	if (position < 1250) position = 1250;
	if (position > 2500) position = 2500;
	
	// Genera pulso PWM "manual"
	uint8_t i;
	for (i = 0; i < 5; i++) {  // Env?a 5 pulsos para asegurar movimiento
		SERVO_PORT |= (1 << SERVO_PIN);  // Pin alto
		
		// Usar valores fijos para evitar errores de compilaci?n
		if (position <= 1250) {
			_delay_us(1250);
			} else if (position <= 1500) {
			_delay_us(1500);
			} else if (position <= 1750) {
			_delay_us(1750);
			} else if (position <= 2000) {
			_delay_us(2000);
			} else if (position <= 2250) {
			_delay_us(2250);
			} else {
			_delay_us(2500);
		}
		
		SERVO_PORT &= ~(1 << SERVO_PIN); // Pin bajo
		_delay_ms(20);  // Periodo de 20ms (50Hz)
	}
}

// Abre el contenedor
void servo_open(void) {
	if (servo_state == 0) {  // Solo si est? cerrado
		// Usa SERVO_OPEN definido en main.h
		SERVO_PORT |= (1 << SERVO_PIN);  // Pin alto
		_delay_us(SERVO_OPEN);           // Usa la constante directamente
		SERVO_PORT &= ~(1 << SERVO_PIN); // Pin bajo
		_delay_ms(20);                   // Periodo de 20ms (50Hz)
		// Repite para asegurar el movimiento
		SERVO_PORT |= (1 << SERVO_PIN);
		_delay_us(SERVO_OPEN);
		SERVO_PORT &= ~(1 << SERVO_PIN);
		_delay_ms(20);
		SERVO_PORT |= (1 << SERVO_PIN);
		_delay_us(SERVO_OPEN);
		SERVO_PORT &= ~(1 << SERVO_PIN);
		_delay_ms(20);
		
		servo_state = 1;
		servo_open_timestamp = 0;  // Reinicia contador
	}
}

// Cierra el contenedor
void servo_close(void) {
	// Usa SERVO_CLOSED definido en main.h
	SERVO_PORT |= (1 << SERVO_PIN);  // Pin alto
	_delay_us(SERVO_CLOSED);         // Usa la constante directamente
	SERVO_PORT &= ~(1 << SERVO_PIN); // Pin bajo
	_delay_ms(20);                   // Periodo de 20ms (50Hz)
	// Repite para asegurar el movimiento
	SERVO_PORT |= (1 << SERVO_PIN);
	_delay_us(SERVO_CLOSED);
	SERVO_PORT &= ~(1 << SERVO_PIN);
	_delay_ms(20);
	SERVO_PORT |= (1 << SERVO_PIN);
	_delay_us(SERVO_CLOSED);
	SERVO_PORT &= ~(1 << SERVO_PIN);
	_delay_ms(20);
	
	servo_state = 0;
}

// ==================== FUNCIONES ULTRAS?NICAS ====================

// Inicializa los sensores ultras?nicos
void ultrasonic_init(void) {
	// Configura pines TRIG como salidas
	US_DDR |= (1 << US1_TRIG_PIN_NUM) | (1 << US2_TRIG_PIN_NUM) | (1 << US3_TRIG_PIN_NUM);
	
	// Configura pines ECHO como entradas
	US_DDR &= ~((1 << US1_ECHO_PIN_NUM) | (1 << US2_ECHO_PIN_NUM) | (1 << US3_ECHO_PIN_NUM));
	
	// Inicialmente pins TRIG en bajo
	US_PORT &= ~((1 << US1_TRIG_PIN_NUM) | (1 << US2_TRIG_PIN_NUM) | (1 << US3_TRIG_PIN_NUM));
	
	// Configura Timer1 para medici?n de tiempo
	TCCR1A = 0; // Modo normal
	TIMSK |= (1 << TOIE1); // Habilita interrupci?n por desbordamiento
	
	// Habilita interrupciones globales
	sei();
}

// Genera un pulso en el pin TRIG
void trig_pulse(uint8_t trig_pin) {
	US_PORT &= ~(1 << trig_pin); // Asegura que el pin esté en bajo
	_delay_us(4);
	US_PORT |= (1 << trig_pin);  // Establece el pin en alto
	_delay_us(12);               // Pulso de 12µs (garantiza 10µs m?nimos)
	US_PORT &= ~(1 << trig_pin); // Regresa el pin a bajo
}

// Mide la distancia utilizando un sensor ultras?nico
uint16_t measure_distance(uint8_t trig_pin, uint8_t echo_pin) {
	uint32_t pulse_duration, distance_cm;
	uint16_t timeout_counter = 0;
	
	// Deshabilita interrupciones durante la medici?n cr?tica
	cli();
	
	// Genera pulso de disparo
	trig_pulse(trig_pin);
	
	// Reinicia el contador
	TCNT1 = 0;
	timer_overflow_count = 0;
	TCCR1B = (1 << CS10); // Sin prescaler
	
	// Espera hasta que el pin ECHO se ponga en alto o timeout
	while (!(US_PIN & (1 << echo_pin))) {
		if (++timeout_counter > MAX_SENSOR_TIMEOUT) {
			TCCR1B = 0; // Detiene el temporizador
			sei(); // Rehabilita interrupciones
			return BIN_DEPTH_CM; // Retorna valor m?ximo en caso de error
		}
	}
	
	// Reinicia contador para medir duraci?n del pulso
	TCNT1 = 0;
	timer_overflow_count = 0;
	timeout_counter = 0;
	
	// Espera hasta que el pin ECHO se ponga en bajo o timeout
	while (US_PIN & (1 << echo_pin)) {
		if (timer_overflow_count > 5 || ++timeout_counter > MAX_SENSOR_TIMEOUT) {
			TCCR1B = 0; // Detiene el temporizador
			sei(); // Rehabilita interrupciones
			return BIN_DEPTH_CM; // Retorna valor m?ximo en caso de error
		}
	}
	
	// Detiene el temporizador
	TCCR1B = 0;
	
	// Rehabilita interrupciones
	sei();
	
	// Calcula la duraci?n del pulso
	pulse_duration = (timer_overflow_count * 65536UL) + TCNT1;
	
	// Calcula la distancia (Velocidad del sonido = 343 m/s = 34300 cm/s)
	// Distancia = (Tiempo × Velocidad) ÷ 2
	// Para F_CPU = 16MHz, cada ciclo es 0.0625 µs
	distance_cm = pulse_duration * 0.01075; // Valor calibrado para 16MHz
	
	// Limita la distancia al rango v?lido
	if (distance_cm > BIN_DEPTH_CM) {
		distance_cm = BIN_DEPTH_CM;
	}
	
	return (uint16_t)distance_cm;
}

// ==================== FUNCIONES DE UTILIDAD ====================

// Inicializa todos los subsistemas
void system_init(void) {
	// Configura puertos no utilizados como salidas con bajo
	DDRB = 0xFF;
	PORTB = 0x00;
	DDRC = 0xFF;
	PORTC = 0x00;
	
	// Inicializa LCD
	lcd_init();
	
	// Inicializa ADC
	adc_init();
	
	// Inicializa sensores ultras?nicos
	ultrasonic_init();
	
	// Inicializa servo motor
	servo_init();
	
	// Peque?o retraso para que todo se estabilice
	_delay_ms(100);
}

// Calcula el porcentaje de llenado en base a la distancia
uint8_t calculate_fill_percentage(uint16_t distance) {
	uint8_t fill_percentage;
	
	if (distance >= BIN_DEPTH_CM) {
		fill_percentage = 0; // Vac?o
		} else if (distance <= MIN_DISTANCE_CM) {
		fill_percentage = 100; // Lleno
		} else {
		// Calcula porcentaje basado en la distancia
		fill_percentage = 100 - ((distance - MIN_DISTANCE_CM) * 100) / (BIN_DEPTH_CM - MIN_DISTANCE_CM);
	}
	
	return fill_percentage;
}

// Muestra el estado de los contenedores en la pantalla LCD
void display_bin_status(uint8_t fill1, uint8_t fill2, uint16_t dist1, uint16_t dist2) {
	char lcd_buffer[17]; // Buffer para 16 caracteres + null
	
	// L?nea 1: Porcentajes de llenado
	lcd_set_cursor(0, 0);
	snprintf(lcd_buffer, sizeof(lcd_buffer), "B1:%3d%% B2:%3d%%", fill1, fill2);
	lcd_string(lcd_buffer);
	
	// L?nea 2: Distancias
	lcd_set_cursor(1, 0);
	snprintf(lcd_buffer, sizeof(lcd_buffer), "D1:%2dcm D2:%2dcm", dist1, dist2);
	lcd_string(lcd_buffer);
}

// Maneja errores mostrando c?digo en LCD
void handle_error(uint8_t error_code) {
	char error_msg[17]; // Buffer para mensaje de error
	
	lcd_clear();
	lcd_set_cursor(0, 0);
	lcd_string("ERROR");
	
	lcd_set_cursor(1, 0);
	snprintf(error_msg, sizeof(error_msg), "Code: %d", error_code);
	lcd_string(error_msg);
	
	_delay_ms(2000); // Muestra el error por 2 segundos
}

// ==================== FUNCI?N PRINCIPAL ====================

int main(void) {
	uint16_t distance1 = 0, distance2 = 0, distance3 = 0;
	uint8_t fill1_percentage = 0, fill2_percentage = 0;
	uint8_t error_status = ERR_NONE;
	uint16_t servo_timer = 0;
	
	// Inicializa todos los subsistemas
	system_init();
	
	// Muestra mensaje de bienvenida
	lcd_clear();
	lcd_string("Smart Trash Bin");
	lcd_set_cursor(1, 0);
	lcd_string("Initializing...");
	_delay_ms(1000);
	
	// Bucle principal
	while (1) {
		// Mide distancia con sensor 1 (nivel de basura 1)
		distance1 = measure_distance(US1_TRIG_PIN_NUM, US1_ECHO_PIN_NUM);
		_delay_ms(20); // Peque?o retraso entre mediciones (aumentado)
		
		// Mide distancia con sensor 2 (nivel de basura 2)
		distance2 = measure_distance(US2_TRIG_PIN_NUM, US2_ECHO_PIN_NUM);
		_delay_ms(20); // Peque?o retraso entre mediciones
		
		// Mide distancia con sensor 3 (detecci?n de persona)
		distance3 = measure_distance(US3_TRIG_PIN_NUM, US3_ECHO_PIN_NUM);
		
		// Calcula porcentajes de llenado
		fill1_percentage = calculate_fill_percentage(distance1);
		fill2_percentage = calculate_fill_percentage(distance2);
		
		// Muestra informaci?n en LCD
		lcd_clear();
		display_bin_status(fill1_percentage, fill2_percentage, distance1, distance2);
		
		// Control del servo basado en detecci?n de persona
		if (distance3 < PERSON_DISTANCE_CM) {
			// Persona detectada, abrir contenedor
			if (servo_state == 0) {  // Solo si est? cerrado
				servo_open();
				lcd_set_cursor(0, 14);
				lcd_string("OP");  // Indicador de abierto en LCD
			}
			servo_timer = 0;  // Reinicia temporizador
			} else {
			// Sin persona detectada, cerrar después de tiempo
			if (servo_state == 1) {
				servo_timer++;
				if (servo_timer >= (SERVO_OPEN_TIME / MEASUREMENT_DELAY)) {
					servo_close();
					lcd_set_cursor(0, 14);
					lcd_string("CL");  // Indicador de cerrado en LCD
				}
			}
		}
		
		// Espera antes de la siguiente medici?n
		_delay_ms(MEASUREMENT_DELAY);
	}
	
	return 0; // Nunca se alcanza
}