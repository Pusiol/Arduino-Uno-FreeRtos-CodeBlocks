/**
Esta é uma demonstração de uso do FreeRTOS, um sistema de
controle PWM para fonte chaveada, no caso um buck entralaçado. As tasks
*/



#include "FreeRTOS.h"
#include "task.h"
//#include "queue.h"
#include "semphr.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "serial.c"

#define TOPCT 0x2FFF
//============================================================
xSemaphoreHandle uartcall,adccall;
unsigned char bufer[5],pix=0,flag=0,result[4];
unsigned char ladc;
//============================================================
ISR(USART_RX_vect)
{
	bufer[pix]=scan();
	if(bufer[pix++]=='\n'){bufer[pix-1]='\0',flag|=1,pix=0;xSemaphoreGive( uartcall );}
	//print("\nmorto\n");
}
//============================================================
int ref=0,refm=0;
ISR(ADC_vect)
{
//	static float x[2],y[2];
	ladc=ADCH;
	xSemaphoreGive( adccall );
	ibnc(ladc,result);printrec(result);put('\n');

}
//============================================================
ISR(TIMER1_OVF_vect)
{
	ibnc(ladc,result);printrec(result);put('\n');
}
//============================================================
//int ref=0,refm=0;
static void Taskleuart(void* pvParameters)
{
  char sta[]="\nHello\nChange for ";
  //print("S444");
  while (1)
  {
	  xSemaphoreTake(uartcall,  portMAX_DELAY);
	  if(flag==1){
	  print(sta);
	  ref=(unsigned char)cnbi(bufer);
	  ibnc(ref,result);printrec(result);
	  print("? (y/N): ");
	  flag=2;
	  }
	  if(flag==3){
		  if(*bufer=='y'){
			  refm=ref;
			  print("\nAlterado\n");
		  }
		  else print("\nCancelado\n");
		  flag=0;
	  }
	  //print(bufer);
	  //ibnc(ladc,result);printrec(result);put('\n');
	  //ibnc(OCR0B,result);printrec(result);put('\n');
  }
}
//============================================================
static void TaskPi(void* pvParameters)
{
  float x[2],y[2];
  int k,l;
  //print("R333");
  while (1)
  {
	  xSemaphoreTake(adccall, portMAX_DELAY);
	  x[1]=x[0];
	  x[0]= refm - ladc;
	  y[1]=y[0];
ibnc((unsigned int)x[0],result);print("x=");printrec(result);put('\n');
	  y[0] = (x[0]+x[1])/2000.0+y[1];

	  if(y[0]<0)y[0]=0; else if(y[0]>0.35)y[0]=0.35;
      k=TOPCT*y[0];
      l=(TOPCT-k);
ibnc((unsigned int)y[0],result);print("y=");printrec(result);put('\n');
ibnc(k,result);print("k=");printrec(result);put('\n');
ibnc(l,result);print("l=");printrec(result);put('\n');
	  OCR1AH=(char)((k&0xFF00)>>8);
	  OCR1AL=(char)k&0xFF;
	  OCR1BH=(char)((l&0xFF00)>>8);
	  OCR1BL=(char)l&0xFF;

ibnc(OCR1AH,result);print("1=");printrec(result);put('\n');
ibnc(OCR1AL,result);print("2=");printrec(result);put('\n');
ibnc(OCR1BH,result);print("3=");printrec(result);put('\n');
ibnc(OCR1BL,result);print("4=");printrec(result);put('\n');

  }
}
//============================================================
static void TaskBlinkBlueLED(void* pvParameters)
{
  DDRB |= 1<<DDB5;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  //print("T555");
  while (1)
  {
	PORTB |= 1<<PORTB5;
    vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_PERIOD_MS));
    PORTB &=~(1<<PORTB5);
    vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_PERIOD_MS));
  }
}
//============================================================

int main()
{
	vSemaphoreCreateBinary( uartcall );vSemaphoreCreateBinary( adccall );
	My_USART_Init();
//===========================================================
    /*TCCR1A=(1<<COM1A1) | (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0) | (0<<WGM11) | (0<<WGM10);
	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (0<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10);
	TCNT1H=0xFF;
	TCNT1L=0xFF;
	ICR1H=0xFF;
	ICR1L=0xFF;
	OCR1AH=0x00;
	OCR1AL=0xFF;
	OCR1BH=0x00;
	OCR1BL=0xFF;*/
TIMSK1=0b00000001;
TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (1<<COM1B0) | (0/*1*/<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (0<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10);

ICR1H=0x2F;
ICR1L=0xFF;
OCR1AH=0x1F;
OCR1AL=0xFF;
OCR1BH=0x1F;
OCR1BL=0xFF;


	DIDR0=(0<<ADC5D) | (0<<ADC4D) | (0<<ADC3D) | (0<<ADC2D) | (0<<ADC1D) | (0<<ADC0D);
	ADMUX=0b01100001;
	ADCSRA=(1<<ADEN) | (1<<ADSC) | (1<<ADATE) | (0<<ADIF) | (1<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);
	ADCSRB=(1<<ADTS2) | (1<<ADTS1) | (0<<ADTS0);

	DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (1<<DDB2) | (1<<DDB1) | (0<<DDB0);
//===========================================================
	sei();
	//print("P111");
	xTaskCreate(TaskPi, (const portCHAR*) "A", 128, NULL, 3, NULL);
	xTaskCreate(Taskleuart, (const portCHAR*) "B", 128, NULL, 2, NULL);
	xTaskCreate(TaskBlinkBlueLED, (const portCHAR*) "C", 128, NULL, 1, NULL);
	//print("Q222");
	vTaskStartScheduler();

  for (;;);

  return 0;
}



