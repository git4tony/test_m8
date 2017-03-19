#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <string.h>
#include "iocompat.h"
#include "printf.h"

/*

     a
    ---
  f| g |b
    ---
  e|   |c
    ---   -
	 d    h
	 
*/
//CHARECTOR
#define CHAR_0 0x3f
#define CHAR_1 0x06
#define CHAR_2 0x5b
#define CHAR_3 0x4f
#define CHAR_4 0x66
#define CHAR_5 0x6d 
#define CHAR_6 0x7d 
#define CHAR_7 0x07 
#define CHAR_8 0x7f 
#define CHAR_9 0x6f  
   
#define CHAR_A 0x77
#define CHAR_C 0x39
#define CHAR_E 0x79
#define CHAR_F 0x71
#define CHAR_H 0x76
#define CHAR_I 0x30
#define CHAR_L 0x38
#define CHAR_O 0x3f
#define CHAR_P 0x73
#define CHAR_S 0x6d
#define CHAR_b 0x7c
#define CHAR_c 0x58
#define CHAR_d 0x5e
#define CHAR_i 0x10
#define CHAR_n 0x54
#define CHAR_o 0x5c
#define CHAR_r 0x50
#define CHAR_t 0x78
#define CHAR_u 0x1c
#define CHAR_BAR 0x40

unsigned char hex_tab[16] = {
   0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f,
   CHAR_A, CHAR_b, CHAR_C, CHAR_d, CHAR_E, CHAR_F
};

enum { UP, DOWN };

ISR (TIMER1_OVF_vect)
{
    static uint16_t pwm;
    static uint8_t direction;

    switch (direction)
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

    OCR = pwm;
}

void ioinit (void)
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

//async mode 32.768 kHz
void timer2_init(void)
{
    //Disable timer2 interrupts
    TIMSK  &= ~(1<<TOIE2);
    //Enable asynchronous mode
    ASSR  = (1<<AS2);
    //set initial counter value
    TCNT2 = 0;
    //set prescaller 128
    TCCR2 |= (1<<CS22)|(1<<CS00);
    //wait for registers update
    while (ASSR & ((1<<TCN2UB)|(1<<TCR2UB)));
    //clear interrupt flags
    TIFR  |= (1<<TOV2);
    //enable TOV2 interrupt
    TIMSK  |= (1<<TOIE2);

    DDRD |= (1<<PD6);
}

//Overflow ISR
ISR(TIMER2_OVF_vect)
{
    //Toggle pin PD6 every second
    PORTD ^= (1<<PD6);
    asm volatile("nop"::);
    //_delay_us(10);
}

void spi_init(void)
{
   DDRB |= (1<<DDB2) | (1<<DDB3) | (1<<DDB5); //spi pins on port B MOSI SCK,SS outputs
   DDRB &= ~(1<<DDB4);
   //SPSR |= (1<<SPI2X);
   SPCR = (1<<SPE) | (1<<MSTR) | (0<<SPR1) | (0<<SPR0) | (0<<CPOL) | (0<<CPHA);  // SPI enable, Master, f/4, mode 00
}

unsigned char spi_transmit(unsigned char data)
{

   SPDR = data;
   while ( !(SPSR & (1<<SPIF)) )
      ;
   return SPDR;
}

// 19200 @8.00 MHz
void uart_init(void)
{
   UCSRB = 0x00;
   UCSRA = (1<<U2X);
   UCSRC = (1<<URSEL) | 0x06;
   UBRRL = 0x33;
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

#define RCK_BIT PD5
#define RCK_0   PORTD &= ~(1<<RCK_BIT)
#define RCK_1   PORTD |= (1<<RCK_BIT)

void shift(unsigned char *p, unsigned char n)
{
    RCK_0;
    while ( n ) {
        spi_transmit(p[--n]);
    }
    RCK_1;
    _delay_us(1);
    RCK_0;
}

unsigned char shift_buf[4];
unsigned char led_buf[16];

void format_scan(unsigned char *out, unsigned char *in, unsigned char index)
{
    //out column
    out[0] = ~in[index];
    //out row
    out[1] = (1<<index);
}

void scan(void)
{
    unsigned char i;

    for ( i = 0; i < 8; i++ ) {
        format_scan(&shift_buf[0], &led_buf[0], i);
        format_scan(&shift_buf[2], &led_buf[8], i);
        shift(shift_buf, sizeof(shift_buf));
        _delay_ms(39);
        memset(shift_buf, 0, sizeof(shift_buf));
        shift(shift_buf, sizeof(shift_buf));
        _delay_ms(1);
    }
}


int main (void)
{

    ioinit ();

    timer2_init();
    uart_init();
    spi_init();

    //RCK
    PORTD &= ~(1<<PD5);
    DDRD |= (1<<PD5);

    memset(shift_buf, 0, sizeof(shift_buf));

    led_buf[0] = hex_tab[0];
    led_buf[1] = hex_tab[1];
    led_buf[2] = hex_tab[2];
    
    led_buf[3] = 0xFF;
    led_buf[4] = 0x03;

    led_buf[8] = hex_tab[0];
    led_buf[9] = hex_tab[1];
    led_buf[10] = hex_tab[2];
    led_buf[11] = hex_tab[3];

    led_buf[12] = hex_tab[10];
    led_buf[13] = hex_tab[11];
    led_buf[14] = hex_tab[12];

    for (;;) {
        //static unsigned int count = 0;
        //sleep_mode();

        //putchar('A');
        //spi_transmit('U');

        //shift_buf[0] = count;
        //shift(shift_buf, sizeof(shift_buf));
        //printf("%d\n", count++);

        scan();
    }

    return (0);
}
