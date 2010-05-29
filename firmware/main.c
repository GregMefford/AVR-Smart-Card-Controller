/* Name: main.c
 * Author: Greg Mefford
 */

#include <avr/io.h>
#include <util/delay.h>

void init_USART_125k(void) {
  // Baud rate
  // UBRR = f_osc/(16*baud) - 1
  // For 125kHz, UBRR = 9 = 20000000/(16*125000) - 1
  UBRRH = 0;
  UBRRL = 9;
  // Set frame format: 8data, 1stop bit, even parity
  UCSRC = (0<<USBS) | (3<<UCSZ0) | (1<<UPM1);
}

void init_USART_9600(void) {
  // Baud rate
  // UBRR = f_osc/(16*baud) - 1
  // For 9600Hz, UBRR ~= 129.2 = 20000000/(16*9600) - 1
  UBRRH = 0;
  UBRRL = 230;
  // Set frame format: 8data, 1stop bit, even parity
  UCSRC = (0<<USBS) | (3<<UCSZ0) | (1<<UPM1);
}

void init_USART_RX(void) {
  // Enable receiver
  UCSRB = (1<<RXEN);
}

void init_USART_TX(void) {
  // Enable transmitter
  UCSRB = (1<<TXEN);
}

void init_PWM_CLK(void) {
  // Generate a clock on OC0B pin.
  // Compare Output Mode: 
  TCCR0A = (0<<COM0B1) | (1<<COM0B0) | // Toggle OC0B on Compare Match
           (1<<WGM01) | (0<< WGM00);   // Clear Timer on Compare Match
  TCCR0B = (0<<CS02) | (0<<CS01) | (1<<CS00); // IO CLK, no pre-scalar
  OCR0A = 4; // Freq. = 20MHz / 2*(1+ORC0A) = 2MHz
}

void USART_TX( unsigned char byte ) {
  // Wait until transmit buffer is ready
  // Send the byte
  UDR = byte;
  while( !( UCSRA & ( 1 << UDRE ) ) )
    ; // Spin-lock
  // 100us for data transmission and guard time to stay synchronous
  _delay_us(110);
}

unsigned char USART_RX( void ) {
  // Wait until receive buffer is ready
  while( !( UCSRA & ( 1 << RXC ) ) )
    ; // Spin-lock
  // Send the byte
  return UDR;
}

void set_reset( void ) {
  PORTD = PORTD | (1 << PORTD3);
}

void clr_reset( void ) {
  PORTD = PORTD & (~(1 << PORTD3));
}

void set_vpp( void ) {
  PORTD = PORTD | (1 << PORTD4);
}

void clr_vpp( void ) {
  PORTD = PORTD & (~(1 << PORTD4));
}

void set_status( void ) {
  PORTD = PORTD | (1 << PORTD6);
}

void clr_status( void ) {
  PORTD = PORTD & (~(1 << PORTD6));
}

int main(void) {
  DDRD = (1<<DDD6) | // Status output
         (1<<DDD5) | // OC0B output pin
         (1<<DDD4) | // T0 output pin for Vpp signal
         (1<<DDD3);  // INT1 output pin for RESET signal
  DDRB = 0xFF; // Port B all outputs
  init_PWM_CLK();
  init_USART_9600();
  init_USART_RX();

  while(1) {
    set_vpp();
    _delay_ms(10);
    set_reset();
    init_USART_RX();
    _delay_ms(80);
    // Send Protocol Type Select
    USART_TX(0xFF);
    init_USART_TX();
    USART_TX(0x00); // Don't set any parameters
    USART_TX(0xFF); // XOR Checksum to nil the previous bytes
    init_USART_RX();
    _delay_ms(100);
    //USART_TX(0x00);
    //USART_TX(0x88);
    //USART_TX(0x00);
    //USART_TX(0x81);
    //USART_TX(0x08);
    // Send Check MF command
    USART_TX(0x80);
    init_USART_TX();
    USART_TX(0x14);
    USART_TX(0x01);
    USART_TX(0x00);
    USART_TX(0x00);
    init_USART_RX();
    set_status();
    unsigned char c = USART_RX();
    PORTB = (0xF0 & 0x10) | (0x0F & c);
    _delay_us(100);
    PORTB = (0xF0 & 0x20) | (0x0F & (c >> 4));
    _delay_us(100);
    PORTB = 0;
    c = USART_RX();
    PORTB = (0xF0 & 0x30) | (0x0F & c);
    _delay_us(100);
    PORTB = (0xF0 & 0x40) | (0x0F & (c >> 4));
    _delay_us(100);
    PORTB = 0;
    clr_status();
    /*
    if(USART_RX() == 0x90) {
      set_status();
      _delay_us(200);
      clr_status();
    }
    if(USART_RX() == 0x01) {
      set_status();
      _delay_us(200);
      clr_status();
    }
    */
    _delay_ms(25);
    clr_reset();
    clr_vpp();
    _delay_ms(100);
  }
  // Will never reach here
  return 0;
}
