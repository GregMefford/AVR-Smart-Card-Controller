/* Name: main.c
 * Author: Greg Mefford
 */

#include <avr/io.h>
#include <util/delay.h>

typedef struct {
  unsigned char data[16];
  unsigned char length;
} message_t;

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
  // Enable transmitter and tell it the buffer is empty.
  UCSRB = (1<<TXEN) | (1<<TXC);
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
  // Clear the TX Complete flag by writing a 1 to it
  UCSRA = UCSRA | ( 1 << TXC );
  // Wait until transmit buffer is ready
  while( !( UCSRA & ( 1 << UDRE ) ) )
    ; // Spin-lock
  // Send the byte
  UDR = byte;
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

void print_byte(unsigned char byte, unsigned char tag) {
  // Present a byte on PORTB, one half-byte at a time,
  // with a tag to identify it using a logic analyzer
  // The pair of bytes presented on PORTB uses the following structure:
  // MSb                                             LSb
  // .-------------------------------------------------.
  // | Bit 7      | Bit 6       | Bit 5:4    | Bit 3:0 |
  // | Data Valid | Lower/Upper | 0x03 & Tag | Data    |
  // '-------------------------------------------------'
  // If Bit 6 is 0, Data is the least-significant half-byte
  // If Bit 6 is 1, Data is the most-significant half-byte
  // After displaying each half-byte of data for 25us, a 0x00 byte will be
  // displayed so that the Data Valid bit (Bit 7) can be used as a data clock.
  unsigned char data_valid = 0x80;
  unsigned char is_upper   = 0x40;
                tag        = (0x30 & tag) << 4;
  unsigned char byte_upper = 0x0F & (byte >> 4);
                byte       = 0x0F & byte;
  
  PORTB = data_valid | is_upper | tag | byte_upper;
  _delay_us(25);
  PORTB = 0;
  _delay_us(25);
  PORTB = data_valid | tag | byte;
  _delay_us(25);
  PORTB = 0;
  _delay_us(25);
}

message_t send_apdu(message_t apdu) {
  // Go into TX mode
  init_USART_TX();
  // Send the APDU command bytes
  unsigned int i;
  for(i=0; i < 5; i++) {
    USART_TX(apdu.data[i]);
  }
  // Wait until transmit buffer is finished
  while( !( UCSRA & ( 1 << TXC ) ) )
    ; // Spin-lock
  // Get the response
  init_USART_RX();
  message_t response;
  unsigned char c = USART_RX();
  if( (c & 0xF0) == 0x60 || (c & 0xF0) == 0x90 ) {
    // This is a SW1. Looks for a SW2
    response.data[0] = c;
    response.data[1] = USART_RX();
    response.length = 2;
    return response;
  }
  unsigned char ack = c & 0xFE;
  if( (apdu.data[1] & 0xFE) == (c & 0xFE) ) {
    _delay_us(200);
    // Back into TX mode
    init_USART_TX();
    // Send the data
    for(; i < apdu.length; i++) {
      USART_TX(apdu.data[i]);
    }
    // Wait until transmit buffer is finished
    while( !( UCSRA & ( 1 << TXC ) ) )
      ; // Spin-lock
    // Assume we are going to get SW1 SW2 after this
    init_USART_RX();
    response.data[0] = USART_RX();
    response.data[1] = USART_RX();
    response.length = 2;
    return response;
  }
  return response;
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
  
  message_t apdu;
  message_t response;
  while(1) {
    // Power-up procedure
    set_vpp();
    _delay_ms(10);
    set_reset();
    init_USART_RX();
    _delay_ms(80);
    
    // Send Protocol Type Select
    init_USART_TX();
    USART_TX(0xFF);
    USART_TX(0x00);
    USART_TX(0xFF);
    // Wait until transmit buffer is finished
    while( !( UCSRA & ( 1 << TXC ) ) )
      ; // Spin-lock
    init_USART_RX();
    
    _delay_ms(10);
    
    // Send Check MF command
    apdu.data[0] = 0x80;
    apdu.data[1] = 0x14;
    apdu.data[2] = 0x01;
    apdu.data[3] = 0x00;
    apdu.data[4] = 0x00;
    apdu.length = 5;
    response = send_apdu(apdu);
    // Expected output: 0x9001
    print_byte(response.data[0], 0x01);
    print_byte(response.data[1], 0x01);
    
    _delay_ms(100);
    
    // Select file 4000
    apdu.data[0] = 0x00;
    apdu.data[1] = 0xA4;
    apdu.data[2] = 0x00;
    apdu.data[3] = 0x00;
    apdu.data[4] = 0x02;
    apdu.data[5] = 0x40;
    apdu.data[6] = 0x00;
    apdu.length = 7;
    response = send_apdu(apdu);
    set_status();
    // Expected output: 0x611B
    print_byte(response.data[0], 0x02);
    print_byte(response.data[1], 0x02);
    clr_status();
    
    _delay_ms(100);
    
    //USART_TX(0x00);
    //USART_TX(0x88);
    //USART_TX(0x00);
    //USART_TX(0x81);
    //USART_TX(0x08);
    
    _delay_ms(25);
    
    // Shutdown procedure
    clr_reset();
    clr_vpp();
    _delay_ms(100);
  }
  // Will never reach here
  return 0;
}
