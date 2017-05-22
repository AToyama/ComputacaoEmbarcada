/************************************************************************
* Rafael Corsi   - Insper
* rafael.corsi@insper.edu.br
*
* Computação Embarcada
*
* 10-PIO-INTERRUPCAO
*
* [ref] http://www.atmel.com/Images/Atmel-42142-SAM-AT03258-Using-Low-Power-Mode-in-SAM4E-Microcontroller_Application-Note.pdf
* [ref] https://www.eecs.umich.edu/courses/eecs373/labs/refs/M3%20Guide.pdf

/*
PARTE 1 - FEITO

possua um handler diferente por botão (BUTTON1, BUTTON2 e BUTTON3) sendo:
– Button 1 : borda de subida
– Button 2 : borda de descida
– Button 3 : borda de subida e descida
Configure também os três LEDs da placa (LED1,LED2 e LED 3) para mudarem de
estado sempre que entrarem na interrupção de seus respectivos botões (Button1, Button2,
Button3).

PARTE 2 - NÃO TENHO CERTEZA QUE ESTÁ CORRETO
Ative o sleep_mode, e faça o CORE entrar em modo de low power enquanto não tiver
eventos a ser processado.

PARTE 3 - FEITO

*/



/************************************************************************/

#include "asf.h"
#include "conf_clock.h"

/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/**
 * LEDs - placa principal
 */
#define LED_PIO_ID		ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		    8
#define LED_PIN_MASK    (1<<LED_PIN)

/**
 * LEDs - placa OLED
 */

	/*led 1*/
	#define OLED1_PIO_ID		ID_PIOA
	#define OLED1_PIO			PIOA
	#define OLED1_PIN			0
	#define OLED1_PIN_MASK		(1<<OLED1_PIN)
	
	/*led 2*/
	#define OLED2_PIO_ID		ID_PIOC
	#define OLED2_PIO			PIOC
	#define OLED2_PIN			30
	#define OLED2_PIN_MASK		(1<<OLED2_PIN)
	
	/*led 3*/
	#define OLED3_PIO_ID		ID_PIOB
	#define OLED3_PIO			PIOB
	#define OLED3_PIN			2
	#define OLED3_PIN_MASK		(1<<OLED3_PIN)
	
/**
 * Botão - placa principal
 */
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

/**
 * Botões - OLED
 */

	/*BOTÃO 1*/
	#define BUT1_PIO_ID			ID_PIOD
	#define BUT1_PIO			PIOD	
	#define BUT1_PIN			28
	#define	BUT1_PIN_MASK		(1<<BUT1_PIN)
			
	/*BOTÃO 2*/
	#define BUT2_PIO_ID			ID_PIOC
	#define BUT2_PIO			PIOC	
	#define BUT2_PIN			31
	#define	BUT2_PIN_MASK		(1<<BUT2_PIN)
	
	/*BOTÃO 3*/
	#define BUT3_PIO_ID			ID_PIOA
	#define BUT3_PIO			PIOA
	#define BUT3_PIN			19
	#define	BUT3_PIN_MASK		(1<<BUT3_PIN)
	
/************************************************************************/
/* VARIÁVEIS                                                              */
/************************************************************************/
uint32_t sleep = 1;
uint32_t count = 6;
uint32_t delay = 1000000;
uint32_t timer = 0;

/************************************************************************/
/* prototype                                                             */
/************************************************************************/
void led_init(int estado);
void but_init(void);
void OLED_but_init();
void but_Handler();
void but1_Handler();
void but2_Handler();
void but3_Handler();
void led_blink();

/************************************************************************/
/* Interrupçcões                                                        */
/************************************************************************/

void but_Handler(){
	
	//chama a função que pisca o led principal por 3 segundos
	led_blink();
	sleep = 0;
	
    /*
     *  limpa interrupcao do PIO
     */
    uint32_t pioIntStatus;
    pioIntStatus =  pio_get_interrupt_status(BUT_PIO);
    
   /**
    *  Toggle status led
    */
   if(pio_get_output_data_status(LED_PIO, LED_PIN_MASK)){
	    pio_clear(LED_PIO, LED_PIN_MASK);
   }
   else{
	    pio_set(LED_PIO,LED_PIN_MASK);
   }
	///ativa o modo sleep	
	sleep = 1;
    
}

void but1_Handler(){
  
	//chama a função que pisca o led principal por 3 segundos
	led_blink();
	//desabilita o modo sleep
	sleep = 0;
  
    uint32_t pioIntStatus;
    pioIntStatus =  pio_get_interrupt_status(BUT1_PIO);
	
	//altera o estado led respectivo
	if(pio_get_output_data_status(OLED1_PIO, OLED1_PIN_MASK))
	pio_clear(OLED1_PIO, OLED1_PIN_MASK);
	else
	pio_set(OLED1_PIO,OLED1_PIN_MASK);
	
	//ativa o modo sleep
	sleep = 1;
}

void but2_Handler(){
	
	//chama a função que pisca o led principal por 3 segundos
	led_blink();
	
	//desabilita o modo sleep
	sleep = 0;
	
    uint32_t pioIntStatus;
    pioIntStatus =  pio_get_interrupt_status(BUT2_PIO);
	
	//altera o estado led respectivo
	if(pio_get_output_data_status(OLED2_PIO, OLED2_PIN_MASK))
	pio_clear(OLED2_PIO, OLED2_PIN_MASK);
	else
	pio_set(OLED2_PIO,OLED2_PIN_MASK);
	//ativa o modo sleep
	sleep = 1;
}

void but3_Handler(){
	
	//chama a função que pisca o led principal por 3 segundos
	led_blink();
	
	//desabilita o modo sleep
	sleep = 0;

    uint32_t pioIntStatus;
    pioIntStatus =  pio_get_interrupt_status(BUT3_PIO);
	
	//altera o estado led respectivo
	if(pio_get_output_data_status(OLED3_PIO, OLED3_PIN_MASK)){
		pio_clear(OLED3_PIO, OLED3_PIN_MASK);
	}
	else{
		pio_set(OLED3_PIO,OLED3_PIN_MASK);
	}
	//ativa o modo sleep
	sleep = 1;

}

/************************************************************************/
/* Funções	                                                            */
/************************************************************************/

//função que faz o led piscar por três segundos
void led_blink(){
	
	//loop com contador para repetir o blink
	while(count>0){
		
		//a partir daqui realiza o blink 1 vez
		LED_PIO->PIO_SODR =  LED_PIN_MASK;
		
		while(timer<delay){
			
			timer++;
		};
		timer = 0 ;
		
		LED_PIO->PIO_CODR = LED_PIN_MASK;

		while(timer<delay){
			timer++;
		};
		timer = 0 ;
		count --;
	}
	count = 3;
}

/**
 * @Brief Inicializa o pino do LED
 */
void led_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_set_output(LED_PIO, LED_PIN_MASK, 1, 0, 0 );
	
	pmc_enable_periph_clk(OLED1_PIO_ID);
	pio_set_output(OLED1_PIO, OLED1_PIN_MASK, 1, 0, 0 );
	
	pmc_enable_periph_clk(OLED2_PIO_ID);
	pio_set_output(OLED2_PIO, OLED2_PIN_MASK, 1, 0, 0 );
	
	pmc_enable_periph_clk(OLED3_PIO_ID);
	pio_set_output(OLED3_PIO, OLED3_PIN_MASK, 1, 0, 0 );
};

/**
 * @Brief Inicializa o pino do BUT
 *  config. botao em modo entrada enquanto 
 *  ativa e configura sua interrupcao.
 */
void but_init(void){
    /* config. pino botao em modo de entrada */
    pmc_enable_periph_clk(BUT_PIO_ID);
    pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    
    /* config. interrupcao em borda de descida no botao do kit */
    /* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
    pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
    pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, but_Handler);
    
    /* habilita interrupçcão do PIO que controla o botao */
    /* e configura sua prioridade                        */
    NVIC_EnableIRQ(BUT_PIO_ID);
    NVIC_SetPriority(BUT_PIO_ID, 1);
};

void OLED_but_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	
	pio_set_input(BUT1_PIO, BUT1_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT2_PIO, BUT2_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT3_PIO, BUT3_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	//chama o handler respectivo ao botão quando houver interrupção
	pio_enable_interrupt(BUT1_PIO, BUT1_PIN_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIN_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIN_MASK);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIN_MASK, PIO_IT_RISE_EDGE, but1_Handler);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIN_MASK, PIO_IT_FALL_EDGE, but2_Handler);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIN_MASK, PIO_IT_RE_OR_HL, but3_Handler);
	
	/* habilita interrupçcão do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 1);
	
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 1);
	
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 1);
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
	/* Inicializao I/OS                                                     */
	/************************************************************************/
	led_init(1);
    but_init();
	OLED_but_init();

	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	while(1){
		if(sleep){
			/* entra em modo sleep */
			pmc_sleep(SLEEPMGR_SLEEP_WFI);	
		}
       
	};
}


