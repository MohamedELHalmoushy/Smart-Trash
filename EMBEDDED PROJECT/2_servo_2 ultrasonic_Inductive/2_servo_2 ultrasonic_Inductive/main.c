#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL

// Pins
#define SERVO_PIN PD5        // SG90 Servo connected to PD5
#define METAL_SENSOR_PIN PD2 // Inductive Proximity Sensor connected to PD2

// Ultrasonic Sensor Pins
#define TRIG_PIN PD6   // TRIG connected to PD6
#define ECHO_PIN PD0   // ECHO connected to PD7

// New Servo and Ultrasonic Pins
#define SERVO2_PIN PD3       // MG995 Full Metal Servo connected to PD3
#define TRIG2_PIN PC0        // A0
#define ECHO2_PIN PC1        // A1

// Movement time constants for MG995 Full Metal continuous rotation servo
#define OPEN_TIME_MS 1000    // Time to open the trash bin (adjust as needed)
#define CLOSE_TIME_MS 1000   // Time to close the trash bin (adjust as needed)

// Function to send pulse to SG90 servo to the right (wider angle)
void servo_right() {
	for (int i = 0; i < 50; i++) {
		PORTD |= (1 << SERVO_PIN);    // Servo pulse on
		_delay_us(2300);              // 2.3ms pulse width (further right position)
		PORTD &= ~(1 << SERVO_PIN);   // Servo pulse off
		_delay_ms(17.7);              // Complete 20ms cycle
	}
}

// Function to send pulse to SG90 servo to the left (wider angle)
void servo_left() {
	for (int i = 0; i < 50; i++) {
		PORTD |= (1 << SERVO_PIN);    // Servo pulse on
		_delay_us(700);               // 0.7ms pulse width (further left position)
		PORTD &= ~(1 << SERVO_PIN);   // Servo pulse off
		_delay_ms(19.3);              // Complete 20ms cycle
	}
}

// Function to send pulse to SG90 servo to the center
void servo_center() {
	for (int i = 0; i < 50; i++) {
		PORTD |= (1 << SERVO_PIN);    // Servo pulse on
		_delay_us(1500);              // 1.5ms pulse width (center position)
		PORTD &= ~(1 << SERVO_PIN);   // Servo pulse off
		_delay_ms(18.5);              // Complete 20ms cycle
	}
}

// Function to measure the distance using the Ultrasonic sensor
uint16_t measure_distance() {
	// Send a pulse to trigger the ultrasonic sensor
	PORTD |= (1 << TRIG_PIN);     // TRIG high
	_delay_us(10);                // Wait for 10us
	PORTD &= ~(1 << TRIG_PIN);    // TRIG low

	// Wait for ECHO to go high
	while (!(PIND & (1 << ECHO_PIN))) {}

	// Start the timer for ECHO high pulse
	uint16_t start_time = 0;
	while (PIND & (1 << ECHO_PIN)) {
		start_time++;   // Increment counter while ECHO is high
		_delay_us(1);   // Delay 1 microsecond for accurate time
	}
	
	// The time is the duration of ECHO high signal in microseconds
	// Calculate distance in centimeters (speed of sound = 343 m/s or 0.0343 cm/µs)
	uint16_t distance = start_time * 0.0343 / 2;   // Divide by 2 for round trip

	return distance;  // Return the distance in centimeters
}

// ---- MODIFIED CODE FOR MG995 FULL METAL CONTINUOUS ROTATION SERVO ----

// Function to stop the MG995 Full Metal continuous rotation servo
void servo2_stop() {
	for (int i = 0; i < 50; i++) {
		PORTD |= (1 << SERVO2_PIN);
		_delay_us(1500);           // 1.5ms pulse width (stop position for continuous rotation)
		PORTD &= ~(1 << SERVO2_PIN);
		_delay_ms(18.5);           // Complete 20ms cycle
	}
}

// Function to rotate MG995 Full Metal servo clockwise (to open the trash bin)
void servo2_open_trash() {
	// Rotate clockwise at moderate speed for a set time
	uint16_t start_time = 0;
	while (start_time < OPEN_TIME_MS) {
		PORTD |= (1 << SERVO2_PIN);
		_delay_us(1700);           // >1.5ms pulse width (clockwise rotation)
		PORTD &= ~(1 << SERVO2_PIN);
		_delay_ms(18.3);           // Complete 20ms cycle
		start_time += 20;          // Increment by cycle time
	}
	
	// Stop the servo after reaching the desired position
	servo2_stop();
}

// Function to rotate MG995 Full Metal servo counter-clockwise (to close the trash bin)
void servo2_close_trash() {
	// Rotate counter-clockwise at moderate speed for a set time
	uint16_t start_time = 0;
	while (start_time < CLOSE_TIME_MS) {
		PORTD |= (1 << SERVO2_PIN);
		_delay_us(1300);           // <1.5ms pulse width (counter-clockwise rotation)
		PORTD &= ~(1 << SERVO2_PIN);
		_delay_ms(18.7);           // Complete 20ms cycle
		start_time += 20;          // Increment by cycle time
	}
	
	// Stop the servo after reaching the desired position
	servo2_stop();
}

// ------------------------
// New Ultrasonic Measurement
uint16_t measure_distance2() {
	// Send pulse
	PORTC |= (1 << TRIG2_PIN);
	_delay_us(10);
	PORTC &= ~(1 << TRIG2_PIN);

	// Wait for echo high
	while (!(PINC & (1 << ECHO2_PIN))) {}

	uint16_t duration = 0;
	while (PINC & (1 << ECHO2_PIN)) {
		duration++;
		_delay_us(1);
	}

	uint16_t distance = duration * 0.0343 / 2;
	return distance;
}

int main(void) {
	// Set Servo pin as output
	DDRD |= (1 << SERVO_PIN);
	
	// Set Inductive Proximity Sensor pin as input
	DDRD &= ~(1 << METAL_SENSOR_PIN);
	
	// Set Ultrasonic TRIG and ECHO pins
	DDRD |= (1 << TRIG_PIN);    // TRIG as output
	DDRD &= ~(1 << ECHO_PIN);   // ECHO as input
	
	// Enable pull-up resistor on the sensor pin (optional but can help if sensor is not giving enough signal)
	PORTD |= (1 << METAL_SENSOR_PIN);
	
	// New Servo and Ultrasonic Configs
	DDRD |= (1 << SERVO2_PIN);       // Output for second servo
	DDRC |= (1 << TRIG2_PIN);        // Output for TRIG2 (A0)
	DDRC &= ~(1 << ECHO2_PIN);       // Input for ECHO2 (A1)

	// Make sure MG995 Full Metal servo is in closed position at start
	servo2_close_trash();

	// Variable to track if trash bin is open
	uint8_t trash_is_open = 0;

	while (1) {
		// ---------- First part: First servo (SG90) ----------
		uint16_t distance1 = measure_distance();  // From first ultrasonic

		if (distance1 < 3) {
			if (PIND & (1 << METAL_SENSOR_PIN)) {
				servo_right();
				_delay_ms(500);
				servo_center();
				} else {
				servo_left();
				_delay_ms(500);
				servo_center();
			}
			_delay_ms(500);  // Small delay before re-measuring
		}

		// ---------- Second part: Second servo (MG995 Full Metal) ----------
		uint16_t distance2 = measure_distance2();  // From second ultrasonic

		if (distance2 < 5) {
			if (!trash_is_open) {
				servo2_open_trash();
				trash_is_open = 1;
			}
			} else {
			if (trash_is_open) {
				servo2_close_trash();
				trash_is_open = 0;
			}
		}
		
		_delay_ms(100);  // Small delay before next check
	}

	return 0;
}