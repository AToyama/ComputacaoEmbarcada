/**************************************************************************
* Rafael Corsi   - Insper 
* rafael.corsi@insper.edu.br        
*
* Computa��o Embarcada
*
* 08-PIO-ENTRADA
*Parte 1: Utilizar o bot�o reservado ao usu�rio para ativar o LED
*         (Bot�o n�o pressionado = LED acesso/ Bot�o pressionado = Led apagado)

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

	
	while(1){
		
		if((BUT_PIO->PIO_PDSR & BUT_PIN_MASK)==0){				//SE o bot�o do usu�rio est� pressionado (0 -> chave fechada/1 -> chave aberta)
						LED_PIO->PIO_CODR = LED_PIN_MASK;	    //Ativa o LED      
		}
		else{
			LED_PIO->PIO_SODR = LED_PIN_MASK;	//sen�o, apaga o led
		}
		
	  
	};
}


