/*
 Firmware da placa de aquecimento baseada no forno de solda
 
 Atribuições de entradas e saídas:
 PB0 (D8):	MISO do sensor de temperatura
 PB1 (D9):	CS do sensor de temperatura
 PB2 (D10): SCLK do sensor de temperatura
 PB3 (D11):	Controle CA
 PB4 (D12):	Sincronização CA
 PB5 (D13):	LED integrado
 */

#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>

#define BAUD 9600
#include <util/setbaud.h>

#include <util/delay.h>

#define T_SET 100 //Temperatura a manter


// Segundo estágio do temporizador:
volatile uint16_t timer0_overflow_count = 0;


void uart_transmit(uint8_t c) {
    loop_until_bit_is_set(UCSR0A, UDRE0); //Aguardar a disponibilidade da UART
    UDR0 = c;
    loop_until_bit_is_set(UCSR0A, TXC0); //Aguardar o término da transmissão
}


// ~16 µs
uint8_t sspi_read() {
    uint8_t d = 0;
    
    for (int i = 7; i >= 0; i--) {
        PORTB &= ~(_BV(PORTB2));
        _delay_us(1);
        if (PINB & _BV(PINB0))
            d |= (1 << i);
        PORTB |= _BV(PORTB2);
        _delay_us(1);
    }
    
    return d;
}


// ~33 µs
double ts_read() {
    uint16_t t;
    
    PORTB &= ~(_BV(PORTB1));
    _delay_us(1);
    t = sspi_read();
    t <<= 8;
    t |= sspi_read();
    PORTB |= _BV(PORTB1);
    if (t & 0x4)
        return NAN; //(-100) Sensor não conectado
    t >>= 3;
    
    return (double)t * 0.25;
}


// Formatado para o Grapher
void print_process_data(uint32_t time, double temp) {
    char buffer[12];
    
    itoa(time, buffer, 10);
    for (int i=0; buffer[i] != '\0'; i++)
        uart_transmit(buffer[i]);
    uart_transmit('\t');
    dtostrf(temp, 5, 2, buffer);
    for (int i=0; buffer[i] != '\0'; i++)
        uart_transmit(buffer[i]);
    uart_transmit('\n');
    uart_transmit('\r');
}


// A cada 128 µs
ISR(TIMER0_OVF_vect) {
    timer0_overflow_count++;
}


// Máximo de 8,3886075 s
uint32_t elapsed_microseconds() {
    return ((((uint32_t)(timer0_overflow_count)) << 8) + TCNT0) >> 1 /*÷2*/; //Precisão de ±0,5 µs
}


void reset_timer() {
    timer0_overflow_count = 0;
    TCNT0 = 0;
}


void main(void) {
    // Configurar entradas...
    DDRB &= ~(_BV(DDB4));
    //    PORTB |= _BV(PORTB4); //Habilitar pull-up
    // e saídas
    DDRB |= _BV(DDB3);
    DDRB |= _BV(DDB5);
    
    // Configurar USART em modo assíncrono...
    UCSR0C &= ~(_BV(UMSEL00));
    UCSR0C &= ~(_BV(UMSEL01));
    // dados de 8 bits...
    UCSR0B |= _BV(UCSZ00);
    UCSR0B |= _BV(UCSZ01);
    UCSR0B &= ~(_BV(UCSZ02));
    // 1 bit de parada...
    UCSR0C &= ~(_BV(USBS0));
    // e sem bit de paridade
    UCSR0C &= ~(_BV(UPM00));
    UCSR0C &= ~(_BV(UPM01));
    // Ajustar a velocidade da UART...
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif
    // e habilitar RX e TX
    UCSR0B |= _BV(RXEN0);
    UCSR0B |= _BV(TXEN0);
    
    // Configurar e inicializar o temporizador:
    TIMSK0 |= _BV(TOIE0); //Habilitar interrupção por transbordamento de TCNT0
    sei(); //Habilitar interrupções
    TCCR0B |= _BV(CS01); //Ativar o temporizador com resolução de 0,5 µs
    
    // Configurar sensor de temperatura...
    DDRB &= ~(_BV(DDB0));	//Entrada MISO
    DDRB |= _BV(DDB1);		//Saída CS
    DDRB |= _BV(DDB2);		//Saída SCLK
    PORTB |= _BV(PORTB1);
    // e aguardar sua estabilização
    _delay_ms(500);
    
    // Laço executivo
    uint32_t t = 0;			//Tempo do processo
    double cur_temp;		//Temperatura atual
    uint8_t s_msg;			//Byte recebido pela interface serial
    int8_t p_running = 0;	//Indicador do processo de aquecimento ativo
    while (1) {
        reset_timer(); //Iniciar temporização
        
        if (p_running) {
            if (p_running == -1) {
                // Iniciando processo de aquecimento
                t = 0;
                p_running = 1;
            }
            cur_temp = ts_read();
            print_process_data(t, cur_temp);
            if (cur_temp < T_SET) {
                // Temperatura abaixo do valor ajustado
                if (!(PINB & _BV(PINB3)))
                    PORTB |= _BV(PORTB3); //Ligar aquecimento
            } else if (PINB & _BV(PINB3)) {
                PORTB &= ~(_BV(PORTB3)); //Desligar aquecimento
            }
        }
        
        // Verificar recebimento de mensagem
        if (UCSR0A & _BV(RXC0)) {
            s_msg = UDR0; //Ler a mensagem
            if (s_msg == ' ') {
                if (p_running) {
                    // Encerrar o processo de aquecimento
                    if (PINB & _BV(PINB3))
                        PORTB &= ~(_BV(PORTB3)); //Desligar aquecimento
                    p_running = 0;
                }
                else
                    p_running = -1; //Iniciar o processo no ciclo seguinte
            }
        }
        
        while (elapsed_microseconds() < 1000000); //Aguardar 1 s
        t++;
    }
}
