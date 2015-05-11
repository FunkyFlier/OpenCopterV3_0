#ifndef LED_H
#define LED_H

#include <Arduino.h>


//GREEN pin 42 PL7
#define GreenLEDOutput() DDRL |= 1<<7
#define GreenLEDHigh() PORTL |= 1<<7
#define GreenLEDLow() PORTL &= ~(1<<7)
#define GreenLEDToggle() PORTL ^= (1<<7)

//YELLOW pin 40 PG1
#define YellowLEDOutput() DDRG |= 1<<1
#define YellowLEDHigh() PORTG |= 1<<1
#define YellowLEDLow() PORTG &= ~(1<<1)
#define YellowLEDToggle() PORTG ^= (1<<1)

//BLUE pin 13 PB7
#define BlueLEDOutput() DDRB |= 1<<7
#define BlueLEDHigh() PORTB |= 1<<7
#define BlueLEDLow() PORTB &= ~(1<<7)
#define BlueLEDToggle() PORTB ^= (1<<7)

//RED pin 38 PD7
#define RedLEDOutput() DDRD |= 1<<7
#define RedLEDHigh() PORTD |= 1<<7
#define RedLEDLow() PORTD &= ~(1<<7)
#define RedLEDToggle() PORTD ^= (1<<7)

void LEDInit();
void ControlLED(uint8_t);
#endif
