/**************************************************************************
* Rafael Corsi   - Insper 
* rafael.corsi@insper.edu.br        
*
* Computação Embarcada
*
* 08-PIO-ENTRADA 
* PARTE 3:
* Utilizar a placa : OLED1XPlained que possui mais 3 botões para implementar as
* seguintes funcionalidades:
* 	1. Button 1 : Diminuir	 a frequência do piscar do led
* 	2. Button 3 : Aumentar a frequência do piscar do led
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
 * Botão
 */ 
#define BUT_PIO_ID		ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN			11			
#define BUT_PIN_MASK	(1<<BUT_PIN)


// Botões OLED

#define BUT1_PIO_ID     ID_PIOD
#define BUT1_PIO        PIOD
#define BUT1_PIN        28
#define BUT1_PIN_MASK   (1<<BUT1_PIN)

#define BUT2_PIO_ID     ID_PIOC
#define BUT2_PIO        PIOC
#define BUT2_PIN        31
#define BUT2_PIN_MASK   (1<<BUT2_PIN)

#define BUT3_PIO_ID     ID_PIOA
#define BUT3_PIO        PIOA
#define BUT3_PIN        19
#define BUT3_PIN_MASK   (1<<BUT3_PIN)

/************************************************************************/
/* Funções	                                                            */
/************************************************************************/

/**
 * @Brief Inicializa o pino do LED
 */
void ledConfig(){
	PMC->PMC_PCER0    = (1<<LED_PIO_ID);	    // Ativa clock do periférico no PMC
	LED_PIO->PIO_PER  = LED_PIN_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	LED_PIO->PIO_OER  = LED_PIN_MASK;           // Ativa saída                      (Output ENABLE register)
};

/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{

	/************************************************************************/
	/* Inicialização básica do uC                                           */
	/************************************************************************/
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	/************************************************************************/
	/* Inicializa perifericos                                               */
	/************************************************************************/
	// Configura LED em modo saída
	ledConfig();
	//OLEDconfig();

// Configura botao
	
	
	PMC->PMC_PCER0    = (1<<BUT_PIO_ID);	    // Ativa clock do periférico no PMC
	BUT_PIO -> PIO_ODR=BUT_PIN_MASK;           //desativando modo output
	BUT_PIO -> PIO_PUER=BUT_PIN_MASK;          //Ativando Pull-Up 
	
//configura botão oled


	PMC->PMC_PCER0    = (1<<BUT1_PIO_ID);	    // Ativa clock do periférico no PMC
	BUT1_PIO -> PIO_ODR=BUT1_PIN_MASK;           //desativando modo output
	BUT1_PIO -> PIO_PUER=BUT1_PIN_MASK;          //Ativando Pull-Up	
	
	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	
	int freq;
	int timer = 0;
	
	while(1){
		
		//enquanto o botão 1 estiver pressinado, deixa a frequencia do led duas vezes mais  lenta
		
		if((BUT1_PIO->PIO_PDSR & BUT1_PIN_MASK)==0){      //if a leitura do PDSR é 0 no PIO do botão 1 da placa OLED (0 -> botão fechado/1 -> botão aberto)
			freq = 4000000;         
		}
		//enquanto o botão 3 estiver pressionado, dobra a velocidade do led
		
		else if((BUT3_PIO->PIO_PDSR & BUT3_PIN_MASK)==0){ // o mesmo do primeiro if, porém para o botão 3 da placa OLED
			freq = 1000000;
		}
		
		else{
			freq = 2000000;
			LED_PIO->PIO_SODR = LED_PIN_MASK;	
		}
		
		PIOC->PIO_SODR =  LED_PIN_MASK;
		
		while(timer<freq){
			
			timer++;
		};
		timer = 0 ;
		PIOC->PIO_CODR = LED_PIN_MASK;

		while(timer<freq){
			timer++;
		};
		timer = 0 ;
	};
}


