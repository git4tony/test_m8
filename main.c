#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "iocompat.h"		/* Note [1] */

enum { UP, DOWN };

ISR (TIMER1_OVF_vect)		/* Note [2] */
{
    static uint16_t pwm;	/* Note [3] */
    static uint8_t direction;

    switch (direction)		/* Note [4] */
    {
        case UP:
            if (++pwm == TIMER1_TOP)
                direction = DOWN;
            break;

        case DOWN:
            if (--pwm == 0)
                direction = UP;
            break;
    }

    OCR = pwm;			/* Note [5] */
}

void
ioinit (void)			/* Note [6] */
{
    /* Timer 1 is 10-bit PWM (8-bit PWM on some ATtinys). */
    TCCR1A = TIMER1_PWM_INIT;
    /*
     * Start timer 1.
     *
     * NB: TCCR1A and TCCR1B could actually be the same register, so
     * take care to not clobber it.
     */
    TCCR1B |= TIMER1_CLOCKSOURCE;
    /*
     * Run any device-dependent timer 1 setup hook if present.
     */
#if defined(TIMER1_SETUP_HOOK)
    TIMER1_SETUP_HOOK();
#endif

    /* Set PWM value to 0. */
    OCR = 0;

    /* Enable OC1 as output. */
    DDROC = _BV (OC1);

    /* Enable timer 1 overflow interrupt. */
    TIMSK = _BV (TOIE1);
    sei ();
}


void spi_init(void)
{
   DDRB |= (1<<DDB2) | (1<<DDB3) | (1<<DDB5); //spi pins on port B MOSI SCK,SS outputs
   DDRB &= ~(1<<DDB4);
   SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0) | (1<<CPOL) | (1<<CPHA);  // SPI enable, Master, f/16
}

unsigned char spi_transmit(unsigned char data)
{

   SPDR = data;
   while ( !(SPSR & (1<<SPIF)) )
      ;
   return SPDR;
}

// 9600 @1.00 MHz
void uart_init(void)
{
   UCSRB = 0x00;
   UCSRA = (1<<U2X);
   UCSRC = (1<<URSEL) | 0x06;
   UBRRL = 0x0C;
   UBRRH = 0x00;
   UCSRB = 0x18;    
}

void uart_transmit(unsigned char data)
{
    while ( !( UCSRA & (1<<UDRE)) )
        ;
    UDR = data;
}

unsigned char uart_recieve(void)
{
    while ( !((UCSRA) & (1<<RXC)) )
        ;
    return UDR;
}

int putchar(int c)
{
    if ( c == '\n' )
        putchar('\r');
    
    uart_transmit(c);
    return c;
}

int main (void)
{

    ioinit ();

    uart_init();
    spi_init();

    for (;;) {
        //sleep_mode();

        putchar('A');
        spi_transmit(0x05);
        _delay_ms(100);
    }

    return (0);
}
