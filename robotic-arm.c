#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/power.h>
#include "zyroskop.h"
#include "lcd.h"
#include <avr/interrupt.h>

unsigned char a=0, t=0;
volatile uint32_t time_loop[7] = {55000,30000,43000,43000,43000,43000,43000}; // wartości jednorazowe ustawiające manipulator na pozycji poczatkowej po starcie programu

volatile int Gx,Gy,Gz;
volatile int rate_x =0, rate_y =0,rate_z =0;
float off_x = 0, off_z = 0, off_y = 0;
float off_pom[] = {0,0};
float gain[] = {1,1,26.6,26.6};
volatile int Gx1 = 0,Gx2 = 0,Gy1 = 0,Gy2 = 0,Gz1 = 0,Gz2 = 0;

volatile int kat_X = 0,kat_Y = 0,kat_Z = 0;


//timer0  taktowanie 50hz dla pwm
ISR(TIMER0_OVF_vect)
{
		t++;
		TCNT0 = 92;

		if(t>=2)   //dodatkowy licznik wydłużający czas cyklu PWM
		{


			TCNT1 = time_loop[a];	//wartosć załadowana do timera
			a++;
			switch(a)
			{
				case 1:	{PORTB |=(1<<0); break;}
				case 2:	{PORTB |=(1<<1); break;}
				case 3:	{PORTB |=(1<<2); break;}
				case 4:	{PORTB |=(1<<3); break;}
				case 5:	{PORTB |=(1<<4); break;}
				case 6:	{PORTB |=(1<<5); break;}
				case 7:	{PORTB |=(1<<6); break;}
			}
			t=0;
			

		}

	

}

//timer1 gaszący PWM
ISR(TIMER1_OVF_vect)
{
	switch(a)
	{
		case 1:	{PORTB &= ~(1<<0); break;}
		case 2:	{PORTB &= ~(1<<1); break;}
		case 3:	{PORTB &= ~(1<<2); break;}
		case 4:	{PORTB &= ~(1<<3); break;}
		case 5:	{PORTB &= ~(1<<4); break;}
		case 6:	{PORTB &= ~(1<<5); break;}
		case 7:	{PORTB &= ~(1<<6); a=0; break;}
	}


}

//timer2 
ISR(TIMER2_OVF_vect)
{

	{
	Gx1 = TWI_read_register(gyroscope_W ,OUT_X_L);
	Gx2 = TWI_read_register(gyroscope_W ,OUT_X_H);

	Gy1 = TWI_read_register(gyroscope_W ,OUT_Y_L);
	Gy2 = TWI_read_register(gyroscope_W ,OUT_Y_H);

	Gz1 = TWI_read_register(gyroscope_W ,OUT_Z_L);
	Gz2 = TWI_read_register(gyroscope_W ,OUT_Z_H);

	Gx = (Gx2 << 8) | (Gx1 & 0xff);
	if(Gx > 32767)				{Gx = -(65535 - Gx);}
	if(Gx < 245 && Gx > -245)	{Gx = 0;}

	Gy = (Gy2 << 8) | (Gy1 & 0xff);
	if(Gy > 32767)				{Gy = -(65535 - Gy);}
	if(Gy < 245 && Gy > -245)	{Gy = 0;}

 	Gz = (Gz2 << 8) | (Gz1 & 0xff);
	if(Gz > 32767)				{Gz = -(65535 - Gz);}
	if(Gz < 245 && Gz > -245)	{Gz = 0;}

	kat_X = kat_X + (0.01526*Gx)*0.01632;
	kat_Y = kat_Y + (0.01526*Gy)*0.01632;
	kat_Z = kat_Z + (0.01526*Gz)*0.01632;

	
	if(kat_X < -90)
	{
		kat_X = -90;
	}
	else if(kat_X > 90)
	{
		kat_X = 90;
	}

	if(kat_Y < -45)
	{
		kat_Y = -45;
	}
	else if(kat_Y > 45)
	{
		kat_Y = 45;
	}

	if(kat_Z < -45)
	{
		kat_Z = -45;
	}
	else if(kat_Z > 45)
	{
		kat_Z = 45;
	}

	}

}

void InitADC()
{
	ADMUX=(1<<REFS0);                         // For Aref=AVcc;
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //Rrescalar div factor =128
}
uint16_t ReadADC(uint8_t ch)
{
   //Select ADC Channel ch must be 0-7
   ch=ch&0b00000111;
   ADMUX = (ADMUX & 0xF8)|ch;

   //Start Single conversion
   ADCSRA|=(1<<ADSC);

   //Wait for conversion to complete
   while(!(ADCSRA & (1<<ADIF)));  //Clear ADIF by writing one to it
   //Note you may be wondering why we have write one to clear it
   //This is standard way of clearing bits in io as said in datasheets.
   //The code writes '1' but it result in setting bit to '0' !!!

   ADCSRA|=(1<<ADIF);

   return(ADC);
}

int main()
{

	DDRA = 0x00;
	DDRB = 0xff;
	DDRD = 0xff;
	//PORTC = 0xff;

/*
	uint16_t potencjometr_lokiec;
	uint16_t potencjometr_nadgarstek;
	uint16_t czujnik_obrotu;
	uint16_t czujnik_chwytania;
*/

	//ustawienia timerów
	TCCR1A = 0x00;
	TCCR1B = (1<<CS10) | (0<<CS11) | (0<<CS12); //x1
	TCCR0 =  (0<<CS00)  | (0<<CS01) | (1<<CS02); //x256
	TCCR2 = (1<<CS20)  | (1<<CS21) | (1<<CS22);

	TIMSK = (1<<TOIE1) | (1<<TOIE0) | (1<<TOIE2);
	TCNT0 = 131;
	TCNT2 = 0;
	sei(); //zezwolenie globalne na przerwania 

	LCD_Initalize(); //inicjalizacja wyświetlacza
    LCD_Clear(); //wyczyszczenie wyświetlacza

    Inicjacja(); //inicjalizacja żyroskopu
	initgyro(); 
	DDRD = 0xff; //ustawienie portu D na wyjście
	InitADC();

    while(1)
     {
    	//PORTD ^= (1 << PD7);

    	time_loop[0] = 27000+14000+(kat_X*155.55); //czas sygnału serwa osi X
		time_loop[1] = 27000+14000+(kat_Y*155.55); //czas sygnału serwa osi Y
		time_loop[2] = 27000+14000+(kat_Z*155.55); //czas sygnału serwa osi Z
		
		// min + (max-min/1023)+kat * ((max-min/1023)/90)
		
		27000 + (55000-27000/1023)*ReadADC(0)
		
		time_loop[3] = (int)(((float)(2700+(ReadADC(0)*(55000-27000/1023))) //-off_pom[2]) *gain[2]); //czas sygnału serwa osi fi1
		time_loop[4] = (int)(((float)(2700+(ReadADC(1)*(55000-27000/1023))) //-off_pom[3]) *gain[3]); //czas sygnału serwa osi fi2
		time_loop[5] = (int)(((float)(200+(ReadADC(2)*14)) //-off_pom[4]) *gain[2]); //czas sygnału serwa osi fi3
		


		LCD_WriteText(kat_X,kat_Y,kat_Z); //wyświetlanie kątów wychylenia manipulatora

        _delay_ms(10);
     }
}
