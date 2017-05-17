/**************************************************************************
* Rafael Corsi   - Insper 
* rafael.corsi@insper.edu.br        
*
* Computa��o Embarcada
*
* 08-PIO-ENTRADA
* PARTE 2:
* Utilize o bot�o para controlar se o LED ir� piscar ou n�o.
************************************************************************/

#include "asf.h"
#include "conf_clock.h"

/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/**
 * LEDs
 */
#define LED_PIO_ID		ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN			8
#define LED_PIN_MASK	(1<<LED_PIN) 

/**
 * Bot�o
 */ 
#define BUT_PIO_ID		ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN			11			
#define BUT_PIN_MASK	(1<<BUT_PIN)


/************************************************************************/
/* Fun��es	                                                            */
/************************************************************************/

/**
 * @Brief Inicializa o pino do LED
 */
void ledConfig(){
	PMC->PMC_PCER0    = (1<<LED_PIO_ID);	    // Ativa clock do perif�rico no PMC
	LED_PIO->PIO_PER  = LED_PIN_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	LED_PIO->PIO_OER  = LED_PIN_MASK;           // Ativa sa�da                      (Output ENABLE register)
};

/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{

	/************************************************************************/
	/* Inicializa��o b�sica do uC                                           */
	/************************************************************************/
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	/************************************************************************/
	/* Inicializa perifericos                                               */
	/************************************************************************/
	
	// Configura LED em modo sa�da
	ledConfig();

	// Configura botao
	
	
	PMC->PMC_PCER0    = (1<<BUT_PIO_ID);	    // Ativa clock do perif�rico no PMC
	BUT_PIO -> PIO_ODR=BUT_PIN_MASK;           //desativando modo output
	BUT_PIO -> PIO_PUER=BUT_PIN_MASK;          //Ativando Pull-Up 	
	
	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/

	const int delay = 1000000; //define o tempo de intervalo do blink do led
	int timer = 0;
	
	while(1){
		
		while((BUT_PIO->PIO_PDSR & BUT_PIN_MASK)==0){	//enquanto o bot�o do usu�rio est� pressionado (0 -> chave fechada/1 -> chave aberta)
			
			//faz o blink do led
			
			PIOC->PIO_SODR =  LED_PIN_MASK; //acende o led
			
			while(timer<delay){	//delay para o blink
				timer++;
			};
			timer = 0 ; //zera o timer
			PIOC->PIO_CODR = LED_PIN_MASK; //acende o led

			while(timer<delay){ // delay para o blink
				timer++;
			};
			
			timer = 0 ; // zera o timer
	}
			
	LED_PIO->PIO_CODR = LED_PIN_MASK;	//enquanto o bot�o nao esta pressionado, mant�m o led acesso normal
	  
	};
}


