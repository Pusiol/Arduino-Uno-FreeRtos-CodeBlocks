/*
 * serial.c
 *
 *  Created on: 8 de jul de 2018
 *      Author: Eric Pusiol
 *
 * Esta � uma biblioteca para usar o perif�rico de
 * comunica��o serial do atmega328p, escrita com base
 * no datasheet do chip.
 *
 */
#include <avr/io.h>

/// A frequ�ncia de clock do processador
#define FOSC 16000000

/// Inicia a uart com o baudrate dado e habilita a interrup��o
/// da uart se a flag estiver setada
void USART_Init( unsigned int baud, char interruption){
unsigned int ubrr=FOSC/16/baud-1;
UBRR0H = (unsigned char)(ubrr>>8);
UBRR0L = (unsigned char)ubrr;
UCSR0B = (interruption<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

/// A fun��o acima simplificada para meus prop�sitos exatos
void My_USART_Init(){
UBRR0L = 16;
UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

/// Envia um caractere
void put(unsigned char data){
while (!( UCSR0A & (1<<UDRE0)));
UDR0 = data;
}

/// Envia uma string
void print(char *data){
for(;*(data);data++){
while (!( UCSR0A & (1<<UDRE0)));
UDR0 = *data;}
}

/// Transforma um inteiro em string
void ibnc(int w,char *bf){
	while(w){
		*bf='0'+w%10;
		bf++;
		w/=10;}
	*(bf)='\0';
}

/// Transforma uma string em inteiro
int cnbi(char *bf){
	int w=0;
	while(*bf){
		w=10*w+(*bf-'0');
		bf++;
		}
	return w;
}

/// Envia uma string de tr�s pra frente
/// Usa recurs�o, ent�o seja moderado ou escreva outra
void printrec(char *vec){
	if(*vec)printrec(vec+1); else return;
	put(*vec);
}

/// L� um caracter de forma bloqueante
unsigned char scan(void){
while (!(UCSR0A & (1<<RXC0)));
return UDR0;
}
/**
void sscan(char *data){
for(;*(data-1)!='\n';data++){
while (!( UCSR0A & (1<<UDRE0)));
*data = UDR0;}
*data = '\0';
}*/

/// L� uma string de forma bloqueante
void sscan(char *data){
for(;*(data-1)!='\n';data++)*data = scan();
*data = '\0';
}

/// Fazia sentido quando escrevi
char correio(){
	return (UCSR0A & (1<<RXC0));
}
