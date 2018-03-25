/*
 * FrequencyGenerator.c
 *
 * Created: 20.09.2017 16:55:37							// TODO: add check for double clicking buttons to add special settings
 * Author : s-del										// maybe with additinional counter for gap between presses
 */

 #define F_CPU 16000000
 //#define DEBUG_MODE		// Don't connect lcd in DEBUG-Mode, just listen to TX-Pin

 #ifndef DEBUG_MODE
 #define HOLD_DOWN_CYCLES 100
 #else
 #define HOLD_DOWN_CYCLES 5
 #endif

 #define PRESC_VAL_1	1
 #define PRESC_VAL_8	2
 #define PRESC_VAL_64	3
 #define PRESC_VAL_256	4
 #define PRESC_VAL_1024	5
 #define PRESC_1	1.0
 #define PRESC_2	8.0
 #define PRESC_3	64.0
 #define PRESC_4	256.0
 #define PRESC_5	1024.0

 #define CR1B(PRESC) (1<<WGM12)|(1<<WGM13)|((PRESC)<<CS10)

 #define MUX_LOW   (1<<MUX0)|(1<<ADLAR)|(0<<REFS0)		// ADMUX register for readout of lower  potentionmeter
 #define MUX_HIGH			 (1<<ADLAR)|(0<<REFS0)		// ADMUX register for readout of higher potentionmeter

 #include <stdio.h>
 #include <avr/io.h>
 #include <util/delay.h>
 #include <avr/interrupt.h>

 #include "lcd.h"
 #include <string.h>

 extern char *itoa(int val, char *s, int radix);

 void setupAdc(void);
 #ifdef DEBUG_MODE
 void setupUART(void);
 int uart_put_byte(uint8_t byte, FILE* stream);
 #else
 #endif

 void printInfo16bitPWM(void);
 void printInfoFreqGen(void);
 void printInfoPulseGen(void);

 void setupFreqGen(void);
 void setupPulseGen(void);
 void setupButtons(void);
 void pollButtons(void);
 void readPotentiometers(void);
 float getPrescF(void);

  uint8_t getIndexOfFirstDigit(char *str);
  uint8_t getIndexOfLastDigit(char *str);


 uint8_t readAdcHigh(void);
 uint8_t readAdcLow(void);
 uint8_t lastElem(const char *str);

 char *trimZeroes(char *str);
 char *appendHz(char *str);
 char *convertPeriodAndAppendUnit(float val, char *str);
 char *appendPercentAndTrim(char *str);
 char *getPrescS(char *dest);

 enum STATES {STATE_OFF, STATE_FREQ_GEN, STATE_PULSE_GEN, STATE_16BIT_PWM, STATE_DEBUG, STATE_LAST};	// Add more
 uint8_t STATE = STATE_OFF;

 enum STATS { SHOW_FREQ, SHOW_PERIOD };
 uint8_t showFreqOrPeriod = SHOW_PERIOD;

 enum TASKS { TASK_NONE, TASK_DO_PULSE_GEN };
 uint8_t TODO = TASK_NONE;

 float PWM_16bitDuty_POT = 0.0; // Duty cycle of timer 1 based on potentiometers (16bit) (from 0 to 1)
 float PWM_16bitDuty_TIM = 0.0; // Actual duty cycle of timer 1 (16bit) (from 0 to 1)

 uint8_t btn0Pressed = 0;
 uint8_t btn1Pressed = 0;
 uint8_t btn0DownCycles = 0;	// a value of >100 indicates that the button has been hold down for >1s
 uint8_t btn1DownCycles = 0;
 uint8_t bothPressed = 0;
 uint8_t hold = 0;
 uint8_t stateChanged = 0;
 //uint8_t somethingPressed = 0;	// unused


 uint8_t prev_low_val;
 uint8_t prev_high_val;

 uint8_t low_val = 0;
 uint8_t high_val = 0;

 char s_temp[64];
 char s_freq[17];
 char s_presc[5];
 char s_period[10];
 char s_pulses[6];

 volatile uint8_t presc = PRESC_VAL_1024;
 volatile uint8_t valuesChanged = 1;
 volatile uint16_t pulse_cnt = 1;

 volatile uint16_t count = 0;
 volatile uint8_t pulse_state = 0;

 int main(void)
 {
	#ifdef DEBUG_MODE
	setupUART();
	#else
	lcd_init(LCD_DISP_ON);
	#endif

	setupAdc();
	setupButtons();
	_delay_ms(10);
	readPotentiometers();
	_delay_ms(10);
    setupFreqGen();
	setupPulseGen();
	//sei();
	STATE = STATE_FREQ_GEN;
    while (1)
    {
		pollButtons();
		readPotentiometers();

		switch(STATE)
		{
		case STATE_OFF:
		{
			break;
		}
		case STATE_PULSE_GEN:
		{
			if(stateChanged)
			{
				presc = TCCR1B & 0x07;	// read prescaler from TCCR1B
			}
			if(valuesChanged)
			{
				if(!hold)
				{
					pulse_cnt = ((uint16_t)high_val << 8) + low_val + 1;	// can wrap around, 0 implies count = 65536
					#ifdef DEBUG_MODE
					printf("CNT: %u TODO: %u\n", count, TODO);
					#endif
				}
				if (TODO == TASK_DO_PULSE_GEN && !count)	// Checking for !count is redundant now since
				{											// we're not using interrupts for this anymore (but is doesnt harm)
					#ifdef DEBUG_MODE
					printf("Count max: %u\n", pulse_cnt);
					#else
					lcd_clrscr();
					lcd_puts("Generating\nPulses");	// Print info on screen
					lcd_gotoxy(15,2);
					if (hold)
					{
						lcd_putc('H');
					}
					#endif

					TIFR1 = (1<<TOV1)|(1<<OCF1A);
					#define PULSE_ON()  PORTB |= (1<<PORTB3)
					#define PULSE_OFF() PORTB &= ~(1<<PORTB3)
					#define CLEAR_OCF1A() TIFR1=(1<<OCF1A)
					do										// Highest frequecy we get with this
					{										// is ~470KHz (at OC1A freq. = 8MHz)
						while(!(TIFR1&(1<<OCF1A)));
						PULSE_ON();
						CLEAR_OCF1A();
						while(!(TIFR1&(1<<OCF1A)));
						PULSE_OFF();
						CLEAR_OCF1A();
						//#ifdef DEBUG_MODE					// This will cause a significant delay
						//printf("CVAL: %u\n", count);		// between pulses, only use in 'emergency' situations
						//#endif
						count++;
					}
					while (count != pulse_cnt);
					count = 0;
					TODO = TASK_NONE;
				}
				valuesChanged = 0;
			}
			printInfoPulseGen();
			break;
		}
		case STATE_FREQ_GEN:
		{
			if(stateChanged)
			{
				presc = TCCR1B & 0x07;	// read prescaler from TCCR1B
			}
			// Freq = F_CPU/(2*presc_n*(OCR1A))
			if(valuesChanged && !hold)
			{
				TCCR1B = CR1B(presc);
				OCR1AH = 0xFF-high_val;	//higher value means lower frequency
				OCR1AL = 0xFF-low_val;	//
				uint16_t newDutyVal = (uint16_t)(PWM_16bitDuty_POT * (float)OCR1A);
				OCR1B = newDutyVal;
				if(newDutyVal == 0)
				{
					TCCR1A &= ~(1<<COM1B1);
				}
				else
				{
					TCCR1A |= (1<<COM1B1);
				}
				PWM_16bitDuty_TIM = (float)OCR1B/(float)OCR1A;
				valuesChanged = 0;
			}
			else if(hold && valuesChanged)
			{
				//Maybe change something?
			}
			/*	Problematic, causes problems
			if (OCR1A < TCNT1)
			{
				TCNT1 = 0xFFFE;
			}
			*/
			printInfoFreqGen();
			break;
		}
		case STATE_16BIT_PWM:
		{
			if(stateChanged)
			{
				presc = TCCR1B & 0x07;	// read prescaler from TCCR1B
			}
			// Freq = F_CPU/(2*presc_n*(OCR1A))
			if(valuesChanged && !hold)
			{
				PWM_16bitDuty_POT = ((float)(((uint16_t)high_val)<<8|(uint16_t)low_val))/65535.0;
				uint16_t newDutyVal = (uint16_t)(PWM_16bitDuty_POT * (float)OCR1A);
				OCR1B = newDutyVal;
				if(newDutyVal == 0)
				{
					TCCR1A &= ~(1<<COM1B1);
				}
				else
				{
					TCCR1A |= (1<<COM1B1);
				}
				PWM_16bitDuty_TIM = (float)OCR1B/(float)OCR1A;
				valuesChanged = 0;
			}
			else if(hold && valuesChanged)
			{
				//Maybe change something?
			}
			printInfo16bitPWM();
			break;
		}
		case STATE_DEBUG:		// Maybe exclude from normal usage?
		{
			#ifndef DEBUG_MODE
			lcd_clrscr();
			lcd_puts("Low: ");
			lcd_puts((const char*)itoa(low_val, s_temp, 10));
			lcd_puts("\nHigh: ");
			lcd_puts(itoa(high_val, s_temp, 10));
			#endif
			break;
		}
		default:;
		}

		if (!hold)
		{
			#ifndef DEBUG_MODE
			if(btn0DownCycles >= HOLD_DOWN_CYCLES)
			{
				lcd_gotoxy(14,2);
				lcd_puts("<-");
			}
			else if(btn1DownCycles >= HOLD_DOWN_CYCLES)
			{
				lcd_gotoxy(14,2);
				lcd_puts("->");
			}
			#else
			if(btn0DownCycles >= HOLD_DOWN_CYCLES)
			{
				printf("<-\n");
			}
			else if(btn1DownCycles >= HOLD_DOWN_CYCLES)
			{
				printf("->\n");
			}
			#endif
		}
		stateChanged = 0;
		#ifndef DEBUG_MODE
		_delay_ms(10);
		#else
		_delay_ms(200);
		#endif
    }
 }

void printInfo16bitPWM()
{
	#ifndef DEBUG_MODE
	lcd_clrscr();
	lcd_puts("Duty:");
	sprintf(s_temp, "%u/%u", OCR1B, OCR1A);
	lcd_puts(s_temp);
	lcd_putc('\n');
	lcd_puts("In %: ");
	if(OCR1A != 0)
		sprintf(s_temp, "%f", 100.0 * PWM_16bitDuty_TIM);
	else
		sprintf(s_temp, "%f", 100.0);
	lcd_puts(appendPercentAndTrim(trimZeroes(s_temp)));	// print duty (%)	100.0 * OCR1B / OCR1A
	lcd_gotoxy(15,2);
	if (hold)
	{
		lcd_putc('H');
	}
	#else
	printf("Timer: %u\n", TCNT1);
	printf("Duty: %u / %u\n", OCR1B, OCR1A);
	printf("High: %u\n", high_val);
	printf("Low: %u\n", low_val);
	#endif
}

void printInfoFreqGen()
{
	sprintf(s_freq, "%f", 16000000.0/(2.0*getPrescF()*((float)OCR1A+1.0)));
	appendHz(trimZeroes(s_freq));

	#ifndef DEBUG_MODE
	lcd_clrscr();
	lcd_puts("Freq:    P: ");			// Maybe add option to display
	lcd_puts(getPrescS(s_presc));		// frequency or period
	lcd_putc('\n');	// "Freq:    P: XXXX"
	lcd_putc(' ');
	lcd_puts(s_freq);
	lcd_gotoxy(15,2);
	if (hold)
	{
		lcd_putc('H');
	}
	#else
	printf("Timer: %u\n", TCNT1);
	printf("OCR1A: %u\n", OCR1A);
	printf("High: %u\n", high_val);
	printf("Low: %u\n", low_val);
	printf("Presc: %s\n", getPrescS(s_presc));
	#endif
}

void printInfoPulseGen()
{
	#ifndef DEBUG_MODE
	lcd_clrscr();
	if (showFreqOrPeriod == SHOW_FREQ)
	{
		lcd_puts(" Freq.: ");
		sprintf(s_freq, "%f", 16000000.0/(2.0*getPrescF()*((float)OCR1A+1.0)));
		appendHz(trimZeroes(s_freq));
		lcd_puts(s_freq);
	}
	else			// SHOW_PERIOD
	{
		lcd_puts(" Per. : ");
		float period = (2.0*getPrescF()*((float)OCR1A+1.0))/16000000.0;
		lcd_puts(convertPeriodAndAppendUnit(period, s_period));
	}
	lcd_putc('\n');	// "Freq:    P: XXXX"
	lcd_puts("Pulses: ");
	if (pulse_cnt == 0)
	{
		strcpy(s_pulses, "65536");
	}
	else
	{
		sprintf(s_pulses, "%u", pulse_cnt);
	}
	lcd_puts(s_pulses);
	lcd_gotoxy(15,2);
	if (hold)
	{
		lcd_putc('H');
	}
	#else
	printf("Freq.: %s \n", s_freq);
	printf("PulseCnt: %u\n", pulse_cnt);
	#endif
}

 void pollButtons()			// both buttons are active low		// Problem in hold mode and button press skips valueschanged
 {
	// TODO: maybe enable mode-change when in hold mode
	if (bothPressed)		// both buttons were pressed previously
	{
		if((PIND & (1<<PIND2)) && (PIND & (1<<PIND3)))		// both buttons released
		{
			bothPressed = 0;
			btn0Pressed = 0;
			btn1Pressed = 0;
			btn0DownCycles = 0;
			btn1DownCycles = 0;
		}
		return;
	}
	else
	{
		if (!((PIND & (1<<PIND2)) || (PIND & (1<<PIND3))))			// both buttons pressed
		{
			if(!bothPressed)
			{
				hold = 1-hold;			//toggle hold state
			}
			bothPressed = 1;
			btn0DownCycles = 0;
			btn1DownCycles = 0;
			return;
		}

		if (btn0Pressed)
		{
			if(PIND & (1<<PIND2))		// Button 0 released
			{
				if (btn0DownCycles<HOLD_DOWN_CYCLES) // Button 0 was not pressed longer than 1sec
				{
					if (!hold)
					{
						if (STATE == STATE_PULSE_GEN)
						{
							if (showFreqOrPeriod == SHOW_FREQ)
							{
								showFreqOrPeriod = SHOW_PERIOD;
							}
							else
							{
								showFreqOrPeriod = SHOW_FREQ;
							}
						}
						else
						{
							if(presc > 1)
								presc--;
						}
						valuesChanged = 1;
					}
				}
				else if(!hold)
				{
					if(STATE >= STATE_LAST)
					{
						STATE = STATE_LAST-1;
					}
					STATE--;					// Set STATE to the previous one
					if(STATE == STATE_OFF)		// Prevent underflow
						STATE = STATE_LAST-1;	//
					stateChanged = 1;
					//valuesChanged = 1;		// Uncomment this to enable immediate update of values after mode has changed
				}
				btn0Pressed = 0;
				btn0DownCycles = 0;
			}
			else
			{
				if(btn0DownCycles<HOLD_DOWN_CYCLES)			// cap off at 100
					btn0DownCycles++;
			}
		}
		else if (!(PIND & (1<<PIND2)))
		{
			btn0Pressed = 1;
		}

		if (btn1Pressed)
		{
			if (PIND & (1<<PIND3))
			{
				if (btn1DownCycles<HOLD_DOWN_CYCLES)
				{
					if (STATE == STATE_PULSE_GEN)
					{
						// Do pulse-generation
						TODO = TASK_DO_PULSE_GEN;
					}
					else if(!hold)
					{
						if(presc < 5)
							presc++;
					}
					valuesChanged = 1;
				}
				else if(!hold)
				{
					STATE++;					// Set STATE to the next one
					if(STATE == STATE_LAST)		// Prevent overflow
						STATE = STATE_OFF+1;
					stateChanged = 1;
					//valuesChanged = 1;		// Uncomment this to enable immediate update of values after mode has changed
				}
				btn1Pressed = 0;
				btn1DownCycles = 0;
			}
			else
			{
				if(btn1DownCycles<HOLD_DOWN_CYCLES)
					btn1DownCycles++;
			}
		}
		else if (!(PIND & (1<<PIND3)))
			btn1Pressed = 1;
	}
 }

void readPotentiometers()
{
	low_val = readAdcLow();
	high_val = readAdcHigh();
	if(low_val != prev_low_val || high_val != prev_high_val)
	{
		prev_high_val = high_val;
		prev_low_val = low_val;
		valuesChanged = 1;
	}
}

 char *getPrescS(char *dest)
 {
	switch(presc)
	{
	case 1:
	{
		strcpy(dest, "1");
		break;
	}
	case 2:
	{
		strcpy(dest, "8");
		break;
	}
	case 3:
	{
		strcpy(dest, "64");
		break;
	}
	case 4:
	{
		strcpy(dest, "256");
		break;
	}
	case 5:
	{
		strcpy(dest, "1024");
		break;
	}
	}
	return dest;
 }

 char *trimZeroes(char *str)
 {
	 uint8_t startIndex = getIndexOfFirstDigit(str);
	 uint8_t i = 0;
	 while(str[i+startIndex] != '\0')
	 {
		 str[i] = str[i+startIndex];
		 i++;
	 }
	 str[i] = str[i+startIndex];				// finish string with '\0'
	 str[getIndexOfLastDigit(str) + 1] = '\0';	// trim ending zeroes
	 return str;
 }

 char *appendPercentAndTrim(char *str)
 {
	 uint8_t i;
	 for (i=0; i<strlen(str); i++)
	 {
		 if(str[i] == '.')
		 {
			str[i+5] = '\0';		// cut the number after four digits behing the dot
			break;
		 }
	 }
	 for (i=0; i<strlen(str); i++)
	 {
		 if(str[i] == '\0')
		 {
			 break;
		 }
	 }
	 str[i]='%';
	 str[i+1]='\0';
	 return str;
 }

 char *appendHz(char *str)
 {
	 uint8_t i;
	 for (i=0;i<strlen(str);i++)
	 {
		 if(str[i] == '\0')
		 {
			 break;
		 }
	 }
	 str[i]=' ';
	 str[i+1]='H';
	 str[i+2]='z';
	 str[i+3]='\0';
	 return str;
 }

 char *convertPeriodAndAppendUnit(float val, char *str)	// val € [0,000000125, 8,3886]
 {
	char temp_unit[3];
	float val_temp;
	if (val <= 0.000001)		// ns
	{
		val_temp = val*1000000000;
		strcpy(temp_unit, "ns");
	}
	else if(val <= 0.001)		// us
	{
		val_temp = val*1000000;
		strcpy(temp_unit, "us");
	}
	else if(val <= 1)			// ms
	{
		val_temp = val*1000;
		strcpy(temp_unit, "ms");
	}
	else						// s
	{
		val_temp = val;
		strcpy(temp_unit, "s");
	}
	sprintf(str, "%f", val_temp);
	uint8_t i;
	for (i=0; i<strlen(str); i++)
	{
		if(str[i] == '.')
		{
			str[i+5] = '\0';		// cut the number after four digits behind the dot
			break;
		}
	}
	strcat(str, temp_unit);
	return str;
 }

 uint8_t getIndexOfFirstDigit(char *str)
 {
	 for (uint8_t i = 0; ;i++)
	 {
		 if(str[i] == '.')
			return i-1;
		 if(str[i] != '0')
		 {
			 return i;
		 }
	 }
	 return -1;
 }

 uint8_t getIndexOfLastDigit(char *str)
 {
	 for (uint8_t i = lastElem(str); i!=0xFF; i--)
	 {
		if(str[i] != '0')
		{
			if(str[i] == '.')
				return i-1;
			return i;
		}
	}
	return -1;
 }

uint8_t lastElem(const char *str)	// returns index of last char (without '\0')
{
	uint8_t curs = 0;
	while (1)
	{
		if(str[curs] != '\0')
			curs++;
		else return curs-1;
	}

}

 void setupFreqGen()
 {
	TCCR1A = (1<<COM1A0)|(1<<COM1B1)|(1<<WGM10)|(1<<WGM11);		// Togggle OC1A on match
	TCCR1B = CR1B(presc);										// Clear Timer on match	(start with prescaler of 1024)
	DDRB |= (1<<DDB1)|(1<<DDB2);								// Frequency-pin & 16bit-PWM-pin output
	TCNT1 = 0xFFFF;
 }

 void setupPulseGen()
 {
	//TIMSK1 |= (1<<OCIE1A);
	//TIFR1 |= (1<<TOV1);
	PORTB &= ~(1<<PORTB3);
	DDRB |= (1<<PORTB3);
 }

 void setupAdc()
 {
	ADMUX = (1<<ADLAR)|(0<<REFS0);							// Analog input at PC0&PC1 and left adjust so that the 8bit value is completely in ADCH
	ADCSRA = (1<<ADEN) |(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2);	// ADPS = 7 =^= prescaler = 128 (not sure if necessary)
	DIDR0 |= (1<<ADC0D)|(1<<ADC1D);							// Disable digital input buffer on PC0&PC1 (only used for Analog input) to reduce power consumption
 }

 void setupButtons()
 {
	DDRD &= ~((1<<DDD2)|(1<<DDD3));
	PORTD |= (1<<PORTD2)|(1<<PORTD3);				// Pullup enable
 }

 uint8_t readAdcLow()
 {
	ADMUX = MUX_LOW;
	ADCSRA |= (1<<ADSC);		// read adc twice because first value after mux change is innaccurate
	while(ADCSRA&(1<<ADSC));
	ADCSRA |= (1<<ADSC);
	while(ADCSRA&(1<<ADSC));
	return 0xFF-ADCH;				// Lower potentiometer is wired up in reverse
 }

 uint8_t readAdcHigh()
 {
	ADMUX = MUX_HIGH;
	ADCSRA |= (1<<ADSC);		// read adc twice because first value after mux change is innaccurate
	while(ADCSRA&(1<<ADSC));
	ADCSRA |= (1<<ADSC);
	while(ADCSRA&(1<<ADSC));
	return ADCH;
 }

float getPrescF()
{
	switch (presc)
	{
	case PRESC_VAL_1:
		return PRESC_1;
	case PRESC_VAL_8:
		return PRESC_2;
	case PRESC_VAL_64:
		return PRESC_3;
	case PRESC_VAL_256:
		return PRESC_4;
	case PRESC_VAL_1024:
		return PRESC_5;
	}
	return 0;	//unreachable
}

/*
#define PULSE_ON()  PORTB |= (1<<PORTB3)
#define PULSE_OFF() PORTB = ~(1<<PORTB3)
ISR(TIMER1_COMPA_vect)
{
	if (pulse_state)
	{
		PULSE_OFF();
		pulse_state = 0;
		count++;
	}
	else
	{
		PULSE_ON();
		pulse_state = 1;
	}
	if (count == pulse_cnt)
	{
		TIMSK1 &= ~(1<<OCIE1A);
		count = 0;
		TODO = TASK_NONE;
		#ifdef DEBUG_MODE
		printf("finished pulse gen!\n");
		#endif
	}
}
*/

//UART STUFF (DEBUG)
#ifdef DEBUG_MODE
FILE mystream = FDEV_SETUP_STREAM( uart_put_byte, NULL, _FDEV_SETUP_WRITE );

void setupUART()
{
	DDRD |= 1<<PORTD1;

	// UART mit 9600 Baud no Parity und 1 Stoppbit

	UBRR0H = 0;
	UBRR0L = 103;
	UCSR0A = 0;
	UCSR0B = (1<<TXEN0)|(1<<RXEN0);
	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);
	stdout = &mystream;
	stdin = &mystream;
}

int uart_put_byte(uint8_t byte, FILE* stream)
{
	while(!(UCSR0A&(1<<UDRE0)));
	UDR0 = byte;
	return 0;
}
#endif