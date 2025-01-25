
#define F_CPU 16000000UL  
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>  


#define LOW  0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1
#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5
#define RS_PORT PORTB
#define RS_DDR  DDRB
#define RS_PIN  PB4

#define EN_PORT PORTB
#define EN_DDR  DDRB
#define EN_PIN  PB3

#define D4_PORT PORTD
#define D4_DDR  DDRD
#define D4_PIN  PD5

#define D5_PORT PORTD
#define D5_DDR  DDRD
#define D5_PIN  PD4

#define D6_PORT PORTB
#define D6_DDR  DDRB
#define D6_PIN  PB5

#define D7_PORT PORTD
#define D7_DDR  DDRD
#define D7_PIN  PD7










void normalCase();
void urgentCase();
void carPass();
void carStop();
void middleStatus();
void normalCase();
void urgentCase();
void pinMode(uint8_t pin, uint8_t mode);
void disablePWM(uint8_t pin);
void digitalWrite(uint8_t pin, uint8_t value);
uint8_t digitalRead(uint8_t pin);
void analogWrite(int pin, int value);
uint16_t analogRead(uint8_t pin) ;
void lcd_pinSetup();
void lcd_enablePulse();
void lcd_send4Bits(uint8_t data);
void lcd_send(uint8_t value, uint8_t mode);
void lcd_init();
void lcd_clear();
void lcd_setCursor(uint8_t col, uint8_t row);
void lcd_print(const char* str);
void lcd_printInt(int value);
void lcd_printFloat(float value, uint8_t decimalPlaces);
void lcd_begin(uint8_t cols, uint8_t rows);
void uartInit(unsigned int baud);
void uartTransmit(char data);
void uartPrint(const char *str) ;
void uartPrintln(const char *str);
void uartPrintFloatln(float value, int precision);
void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), uint8_t mode);
ISR(INT0_vect);
ISR(INT1_vect);
void delay(uint16_t ms);


// LCD command
#define LCD_CLEAR 0x01 // clear screen
#define LCD_HOME  0x02 // cursor reset

const int greenPin = 10;
const int redPin = 9;
const int yellowPin = 6;
//LiquidCrystal lcd(12, 11, 5, 4, 13, 7);
float lowVol = 1.8;
float redVol = 3.0;
float yellowVol = 3.0;
float greenVol = 3.0;
int buttonPin = 2;
const int lightSensorPin = A0;
int buttonState = 0;
int lcdBackPin = 3;
int lcdrelativeVol = 255;
int lowLcdrelativeVol = 5;
float ratio = 0.5;
int redStatus = 0;
int greenStatus = 0;
//volatile uint8_t interruptFlag = 0;


void pinMode(uint8_t pin, uint8_t mode) {
	if (pin < 8) { // Pin D (Digital pins 0-7)
		if (mode == OUTPUT) {
			DDRD |= (1 << pin);
			} else {
			DDRD &= ~(1 << pin);
		}
		} else if (pin < 14) { // Pin B (Digital pins 8-13)
		pin -= 8;
		if (mode == OUTPUT) {
			DDRB |= (1 << pin);
			} else {
			DDRB &= ~(1 << pin);
		}
		} else if (pin < 20) {
		pin -= 14;
		if (mode == OUTPUT) {
			DDRC |= (1 << pin);
			} else {
			DDRC &= ~(1 << pin);
		}
	}
}


void disablePWM(uint8_t pin) {
	if (pin == 5) { 
		
		TCCR0A &= ~(1 << COM0B1); 
		}
		else if (  pin == 6){
			TCCR0A &= ~(1 << COM0A1); 
		}
		
		else if (pin == 9) { 
		TCCR1A &= ~(1 << COM1A1); 
		
		} 
		else if (  pin == 10){
			TCCR1A &= ~(1 << COM1B1); 
		}
		
		else if (pin == 3 ) {
		
		TCCR2A &= ~(1 << COM2B1); 
	}
	else if(pin == 11) {
		TCCR2A &= ~(1 << COM2A1); 
	}
}


void digitalWrite(uint8_t pin, uint8_t value) {
	disablePWM(pin);
	if (pin < 8) { // Pin D
		if (value) {
			PORTD |= (1 << pin);
			} else {
			PORTD &= ~(1 << pin);
		}
		} else if (pin < 14) { // Pin B
		pin -= 8;
		if (value) {
			PORTB |= (1 << pin);
			} else {
			PORTB &= ~(1 << pin);
		}
		} else if (pin < 20) { // Pin C
		pin -= 14;
		if (value) {
			PORTC |= (1 << pin);
			} else {
			PORTC &= ~(1 << pin);
		}
	}
}


uint8_t digitalRead(uint8_t pin) {
	if (pin < 8) { // Pin D
		return (PIND & (1 << pin)) ? 1 : 0;
		} else if (pin < 14) { // Pin B
		pin -= 8;
		return (PINB & (1 << pin)) ? 1 : 0;
		} else if (pin < 20) { // Pin C
		pin -= 14;
		return (PINC & (1 << pin)) ? 1 : 0;
	}
	return 0;
}


void analogWrite(int pin, int value) {
	if (pin == 5 || pin == 6) {
		// Timer0 - Fast PWM Mode
		TCCR0A |= (1 << WGM00) | (1 << WGM01);
		TCCR0B |= (1 << CS01);

		if (pin == 5) {
			TCCR0A |= (1 << COM0B1);
			DDRD |= (1 << PD5);
			OCR0B = value;
			} else if (pin == 6) {
			TCCR0A |= (1 << COM0A1);
			DDRD |= (1 << PD6);
			OCR0A = value;
		}
		} else if (pin == 9 || pin == 10) {
		// Timer1 - Fast PWM Mode (8-bit)
		TCCR1A |= (1 << WGM10);
		TCCR1B |= (1 << WGM12) | (1 << CS11);

		if (pin == 9) {
			TCCR1A |= (1 << COM1A1);
			DDRB |= (1 << PB1);
			OCR1A = value;
			} else if (pin == 10) {
			TCCR1A |= (1 << COM1B1);
			DDRB |= (1 << PB2);
			OCR1B = value;
		}
		} else if (pin == 3 || pin == 11) {
		// Timer2 - Fast PWM Mode
		TCCR2A |= (1 << WGM20) | (1 << WGM21);
		TCCR2B |= (1 << CS21);

		if (pin == 3) {
			TCCR2A |= (1 << COM2B1);
			DDRD |= (1 << PD3);
			OCR2B = value;
			} else if (pin == 11) {
			TCCR2A |= (1 << COM2A1);
			DDRB |= (1 << PB3);
			OCR2A = value;
		}
	}
}


uint16_t analogRead(uint8_t pin) {
	ADMUX = (1 << REFS0) | (pin & 0x07);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}


void lcd_pinSetup() {
	RS_DDR |= (1 << RS_PIN);
	EN_DDR |= (1 << EN_PIN);
	D4_DDR |= (1 << D4_PIN);
	D5_DDR |= (1 << D5_PIN);
	D6_DDR |= (1 << D6_PIN);
	D7_DDR |= (1 << D7_PIN);
}


void lcd_enablePulse() {
	EN_PORT |= (1 << EN_PIN);
	_delay_us(1);
	EN_PORT &= ~(1 << EN_PIN);
	_delay_us(100);
}


void lcd_send4Bits(uint8_t data) {
	if (data & 1) D4_PORT |= (1 << D4_PIN); else D4_PORT &= ~(1 << D4_PIN);
	if (data & 2) D5_PORT |= (1 << D5_PIN); else D5_PORT &= ~(1 << D5_PIN);
	if (data & 4) D6_PORT |= (1 << D6_PIN); else D6_PORT &= ~(1 << D6_PIN);
	if (data & 8) D7_PORT |= (1 << D7_PIN); else D7_PORT &= ~(1 << D7_PIN);
	lcd_enablePulse();
}

void lcd_send(uint8_t value, uint8_t mode) {
	if (mode) RS_PORT |= (1 << RS_PIN);
	else RS_PORT &= ~(1 << RS_PIN);

	lcd_send4Bits(value >> 4);
	lcd_send4Bits(value & 0x0F);
}


void lcd_init() {
	lcd_pinSetup();

	
	_delay_ms(20);
	lcd_send4Bits(0x03);
	_delay_ms(5);
	lcd_send4Bits(0x03);
	_delay_us(150);
	lcd_send4Bits(0x03);
	lcd_send4Bits(0x02);

	
	lcd_send(0x28, 0);
	lcd_send(0x0C, 0);
	lcd_send(0x06, 0);
	lcd_send(LCD_CLEAR, 0);
	_delay_ms(2);
}


void lcd_clear() {
	lcd_send(LCD_CLEAR, 0);
	_delay_ms(2);
}


void lcd_setCursor(uint8_t col, uint8_t row) {
	uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
	lcd_send(0x80 | (col + row_offsets[row]), 0);
}

void lcd_print(const char* str) {
	while (*str) {
		lcd_send(*str++, 1);
	}
}

void lcd_printInt(int value) {
	char buffer[8];
	itoa(value, buffer, 10);
	lcd_print(buffer);
}



void lcd_printFloat(float value, uint8_t decimalPlaces) {
	char buffer[16];
	dtostrf(value, 0, decimalPlaces, buffer);
	lcd_print(buffer);
}


void lcd_begin(uint8_t cols, uint8_t rows) {
	lcd_init();
}


// UART Initialization
void uartInit(unsigned int baud) {
	unsigned int ubrr = F_CPU / 16 / baud - 1;
	UBRR0H = (unsigned char)(ubrr >> 8);  // Set baud rate high byte
	UBRR0L = (unsigned char)ubrr;        // Set baud rate low byte
	UCSR0B = (1 << TXEN0);               // Enable transmitter
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8 data bits, 1 stop bit
}

// Transmit a single character
void uartTransmit(char data) {
	while (!(UCSR0A & (1 << UDRE0)));  // Wait for the transmit buffer to be empty
	UDR0 = data;                       // Load data into the transmit buffer
}

// Transmit a string
void uartPrint(const char *str) {
	while (*str) {
		uartTransmit(*str++);
	}
}



// Transmit a string followed by a newline
void uartPrintln(const char *str) {
	uartPrint(str);
	uartTransmit('\r');
	uartTransmit('\n');
}

// Helper function to print floating-point numbers
void uartPrintFloatln(float value, int precision) {
	char buffer[20];  // Buffer to hold the floating-point number as a string
	dtostrf(value, 1, precision, buffer);  // Convert float to string
	uartPrint(buffer);
	uartTransmit('\r');
	uartTransmit('\n');
}


void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), uint8_t mode) {
	if (interruptNum == 0) { 
		EICRA &= ~((1 << ISC01) | (1 << ISC00)); 
		if (mode == 0) {
			EICRA |= (0 << ISC01) | (0 << ISC00); 
			} else if (mode == 1) {
			EICRA |= (1 << ISC01) | (0 << ISC00);
			} else if (mode == 2) {
			EICRA |= (1 << ISC01) | (1 << ISC00);
			} else if (mode == 3) {
			EICRA |= (1 << ISC01) | (1 << ISC00); 
		}
		EIMSK |= (1 << INT0); 
		} else if (interruptNum == 1) { 
		EICRA &= ~((1 << ISC11) | (1 << ISC10)); 
		if (mode == 0) {
			EICRA |= (0 << ISC11) | (0 << ISC10); 
			} else if (mode == 1) {
			EICRA |= (1 << ISC11) | (0 << ISC10);
			} else if (mode == 2) {
			EICRA |= (1 << ISC11) | (1 << ISC10); 
			} else if (mode == 3) {
			EICRA |= (1 << ISC11) | (1 << ISC10); 
		}
		EIMSK |= (1 << INT1); 
	}
	sei(); 
}



ISR(INT0_vect) {
	urgentCase();
}


ISR(INT1_vect) {
	urgentCase(); 
}

void delay(uint16_t ms) {
	while (ms--) {
		_delay_ms(1); 
	}
}







void carPass(){
	float greenPinVol =lowVol+(greenVol-lowVol)*ratio;
	analogWrite(greenPin, (uint8_t)((greenPinVol/5.0)*255));
	digitalWrite(yellowPin, 0);
	digitalWrite(redPin, 0);
	uartPrintln("GREEN");
	greenStatus = 1;
	redStatus = 0;
}



void carStop(){
	float redPinVol =lowVol+(redVol-lowVol)*ratio;
	analogWrite(redPin, (uint8_t)((redPinVol/5.0)*255));
	digitalWrite(yellowPin, 0);
	digitalWrite(greenPin, 0);
	uartPrintln("RED");
	redStatus = 1;
	greenStatus = 0;
}

void middleStatus(){
	float greenPinVol =lowVol+(greenVol-lowVol)*ratio;
	analogWrite(greenPin, (uint8_t)((greenPinVol/5.0)*255));
	float yellowPinVol =lowVol+(yellowVol-lowVol)*ratio;
	analogWrite(yellowPin,(uint8_t) ((yellowPinVol/5.0)*255));
	for (int i=0; i<100; i++){
		analogWrite(greenPin, (uint8_t)((greenVol/5.0)*255));
		delay(15);
		digitalWrite(greenPin, 0);
		delay(15);
	}
	digitalWrite(yellowPin, 0);
	digitalWrite(greenPin, 0);
}

void normalCase(){
	carPass();
	for (int i = 60; i>0; i--){
		delay(1000);
		lcd_clear();
		lcd_setCursor(0, 0);
		lcd_print("vehicle pass");
		lcd_setCursor(0, 1);
		lcd_printInt(i);
		lcd_print("sec");
	}
	lcd_clear();
	middleStatus();
	carStop();
	for (int i =30; i>0; i--){
		delay(1000);
		lcd_clear();
		lcd_setCursor(0, 0);
		lcd_print("vehicle stop");
		lcd_setCursor(0, 1);
		lcd_printInt(i);
		lcd_print("sec");
	}
}

void urgentCase(){
	buttonState = 0;
	int reading = digitalRead(buttonPin);
	if (reading==1){
		_delay_ms(10);
		int reading_repeat = digitalRead(buttonPin);
		if (reading_repeat == 1){
			uartPrintln("1");
			buttonState = 1;
			lcd_clear();
			lcd_setCursor(0, 0);
			lcd_print("vehicle pass");
			lcd_setCursor(0, 1);
			lcd_print("emergency");
			// analogWrite(greenPin, int((greenVol/5.0)*255));
			analogWrite(greenPin,(uint8_t) (((lowVol+(greenVol-lowVol)*ratio)/5.0)*255));
			digitalWrite(yellowPin, 0);
			digitalWrite(redPin, 0);
			
		}
		_delay_ms(60000);
		if (redStatus == 1){
			float redPinVol =lowVol+(redVol-lowVol)*ratio;
			analogWrite(redPin, (uint8_t)((redPinVol/5.0)*255));
			digitalWrite(greenPin,0);
		}
		if (greenStatus == 1){
			float greenPinVol =lowVol+(greenVol-lowVol)*ratio;
			analogWrite(greenPin, (uint8_t)((greenPinVol/5.0)*255));
			digitalWrite(redPin,0);
		}


	}
}




void setup() {
	uartInit(9600);
	pinMode(greenPin, OUTPUT);
	pinMode(redPin, OUTPUT);
	pinMode(yellowPin, OUTPUT);
	pinMode(lcdBackPin, OUTPUT);
	pinMode(lightSensorPin,INPUT);
	analogWrite(lcdBackPin,(uint8_t) lcdrelativeVol);
	digitalWrite(greenPin, LOW);
	digitalWrite(redPin,LOW);
	digitalWrite(yellowPin, LOW);
	
	lcd_begin(16, 2);
	//pinMode(buttonPin, INPUT); 
	DDRD &= ~(1 << PD2); 
	PORTD |= (1 << PD2); 
	attachInterrupt(0, urgentCase, 2);
}

int main() {
	setup();
	while(1){
		int light = analogRead(lightSensorPin);
		ratio = light/1024.0;
		uartPrint("ratio-");
		uartPrintFloatln(ratio,2);
		analogWrite(lcdBackPin, (uint8_t)(lowLcdrelativeVol+(lcdrelativeVol-lowLcdrelativeVol)*ratio));
		normalCase();		
	}
	return 0;
	}
