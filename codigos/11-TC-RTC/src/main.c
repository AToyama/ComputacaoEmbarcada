#include "asf.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

#define YEAR        2017
#define MOUNTH      3
#define DAY         27
#define WEEK        13
#define HOUR        9
#define MINUTE      5
#define SECOND      0

/**
 * LEDs
 */

/*--------------KIT-------------------------*/

#define LED_PIO_ID		ID_PIOC
#define LED_PIO       PIOC
#define LED_PIN		    8
#define LED_PIN_MASK  (1<<LED_PIN)

/*-------------OLED-------------------------*/

//PIO ID
#define OLED_PIO_ID_led_1	ID_PIOA
#define OLED_PIO_ID_led_2	ID_PIOC
#define OLED_PIO_ID_led_3   ID_PIOB

//PIO
#define OLED_PIO_led_1      PIOA
#define OLED_PIO_led_2      PIOC
#define OLED_PIO_led_3      PIOB

//button ppin
#define OLED_PIN_led_1		0
#define OLED_PIN_led_2		30
#define OLED_PIN_led_3      2

//Mask
#define OLED_PIN_MASK_led_1	(1 << OLED_PIN_led_1)
#define OLED_PIN_MASK_led_2	(1 << OLED_PIN_led_2)
#define OLED_PIN_MASK_led_3 (1 << OLED_PIN_led_3)

/**
 * Botão
 */

/*--------------KIT-------------------------*/

#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

/*-------------OLED-------------------------*/

//PIO ID
#define OLED_PIO_ID_but_1	ID_PIOD
#define OLED_PIO_ID_but_2	ID_PIOC
#define OLED_PIO_ID_but_3   ID_PIOA

//PIO
#define OLED_PIO_but_1         PIOD
#define OLED_PIO_but_2         PIOC
#define OLED_PIO_but_3         PIOA


//button pin
#define OLED_PIN_but_1		28
#define OLED_PIN_but_2		31
#define OLED_PIN_but_3      19


//Mask do botão
#define OLED_PIN_MASK_but_1	(1 << OLED_PIN_but_1)
#define OLED_PIN_MASK_but_2	(1 << OLED_PIN_but_2
#define OLED_PIN_MASK_but_3 (1 << OLED_PIN_but_3)

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/
volatile uint8_t flag_led0 = 1;
volatile uint8_t flag_led1 = 1;
volatile uint8_t flag_led2 = 1;
volatile uint8_t flag_led3 = 1;


/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void BUT_init(void);
void LED_init(int estado);
void TC1_init(void);
void RTC_init(void);
void pin_toggle(Pio *pio, uint32_t mask);
void OLED_led_init(int estado);
void but_OLED_init(void);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

/**
 *  Handle Interrupcao botao 1
 */
static void Button1_Handler(uint32_t id, uint32_t mask)
{
	
}

/**
 *  Interrupt handler for TC1 interrupt. 
 */
void TC0_Handler(void){
	volatile uint32_t ul_dummy;

    /****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
  pin_toggle(LED_PIO, LED_PIN_MASK);
}

/**
 *  Interrupt handler for TC1 interrupt. 
 */
void TC1_Handler(void){
	volatile uint32_t ul_dummy;

    /****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
  pin_toggle(OLED_PIO_led_1, OLED_PIN_MASK_led_1);
}

/**
 *  Interrupt handler for TC1 interrupt. 
 */
void TC2_Handler(void){
	volatile uint32_t ul_dummy;

    /****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC0, 2);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
  pin_toggle(OLED_PIO_led_2, OLED_PIN_MASK_led_2);
}

/**
 *  Interrupt handler for TC1 interrupt. 
 */
void TC3_Handler(void){
	volatile uint32_t ul_dummy;

    /****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC1, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
  pin_toggle(OLED_PIO_led_3, OLED_PIN_MASK_led_3);
}

/**
 * \brief Interrupt handler for the RTC. Refresh the display.
 */
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/* Second increment interrupt */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
	
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);

	} else {
		/* Time or date alarm */
		if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {   
      
      /*Atualiza hora */       
      uint32_t ano,mes,dia,hora,minuto,segundo;
            
      rtc_get_date(RTC, &ano, &mes, &dia, NULL);
      rtc_get_time(RTC, &hora, &minuto, &segundo);
      
      /* incrementa minuto */
      minuto++;
	
	  /* inverte status led */
	  flag_led0 ^= 1;
	
      /* configura novo alarme do RTC */
      rtc_set_date_alarm(RTC, 1, mes, 1, dia);
      rtc_set_time_alarm(RTC, 1, hora, 1, minuto, 1, segundo);
     
	  /* Ativa/desativa o TimerCounter, em seus respectivos canais*/
      if(flag_led0){
		
		tc_start(TC0,0); 
		tc_start(TC0,1); 
		tc_start(TC0,2);
		tc_start(TC1,0);
      }
	  
	  else{
		  
        tc_stop(TC0,0); pio_set(LED_PIO, LED_PIN_MASK);
        tc_stop(TC0,1); pio_set(OLED_PIO_led_1, OLED_PIN_MASK_led_1);
        tc_stop(TC0,2); pio_set(OLED_PIO_led_2, OLED_PIN_MASK_led_2);
        tc_stop(TC1,0); pio_set(OLED_PIO_led_3, OLED_PIN_MASK_led_3);
		
	  }
	  

	  rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
		}
	}
}


/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/** 
 *  Toggle pin controlado pelo PIO (out)
 */
void pin_toggle(Pio *pio, uint32_t mask){
   if(pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
   else
    pio_set(pio,mask);
}

/**
 * @Brief Inicializa o pino do BUT
 */
void BUT_init(void){
    /* config. pino botao em modo de entrada */
    pmc_enable_periph_clk(BUT_PIO_ID);
    pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    
    /* config. interrupcao em borda de descida no botao do kit */
    /* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
    pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
    pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);
    
    /* habilita interrupçcão do PIO que controla o botao */
    /* e configura sua prioridade                        */
    NVIC_EnableIRQ(BUT_PIO_ID);
    NVIC_SetPriority(BUT_PIO_ID, 1);
};

/**
 * @Brief Inicializa o pino do LED
 */
void LED_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
};

/**
 * Configura TimerCounter (TC0) para gerar uma interrupcao no canal 0-(ID_TC1) 
 * a cada 250 ms (4Hz)
 */
void TC_init( Tc *TC, uint32_t ID_TC,  uint32_t channel, uint32_t freq ){   
    uint32_t ul_div;
    uint32_t ul_tcclks;
    uint32_t ul_sysclk = sysclk_get_cpu_hz();
     
    /* Configura o PMC */
    pmc_enable_periph_clk(ID_TC);    

    /** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
    tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
    tc_init(TC, channel, ul_tcclks | TC_CMR_CPCTRG);
    tc_write_rc(TC, channel, (ul_sysclk / ul_div) / freq);

    /* Configura e ativa interrupçcão no TC , 4 canais */
    NVIC_EnableIRQ((IRQn_Type) ID_TC);
    tc_enable_interrupt(TC, channel, TC_IER_CPCS);

    /* Inicializa o canal do TC */
    tc_start(TC, channel);
	
}


//Inicialização LEDs OLED
void OLED_led_init(int estado){
	
	pmc_enable_periph_clk(OLED_PIO_ID_led_1);
	pmc_enable_periph_clk(OLED_PIO_ID_led_2);
	pmc_enable_periph_clk(OLED_PIO_ID_led_3);
	
	pio_set_output(OLED_PIO_led_1, OLED_PIN_MASK_led_1, 1, 0, 0 );
	pio_set_output(OLED_PIO_led_2, OLED_PIN_MASK_led_2, 1, 0, 0 );
	pio_set_output(OLED_PIO_led_3, OLED_PIN_MASK_led_3, 1, 0, 0 );
};


/**
 * Configura o RTC para funcionar com interrupcao de alarme
 */
void RTC_init(){
    /* Configura o PMC */
    pmc_enable_periph_clk(ID_RTC);
        
    /* Default RTC configuration, 24-hour mode */
    rtc_set_hour_mode(RTC, 0);

    /* Configura data e hora manualmente */
    rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
    rtc_set_time(RTC, HOUR, MINUTE, SECOND);

    /* Configure RTC interrupts */
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 0);
    NVIC_EnableIRQ(RTC_IRQn);
    
    /* Ativa interrupcao via alarme */
    rtc_enable_interrupt(RTC,  RTC_IER_ALREN); 
    
}

/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){
  /* Initialize the SAM system */
  sysclk_init();

  /* Disable the watchdog */
  WDT->WDT_MR = WDT_MR_WDDIS;

  /* Configura Leds */
  LED_init(1);  
  OLED_led_init(0);
	
  /* Configura os botões */
  BUT_init();    
  
  
  //incializando em diferentes frequencias
  TC_init(TC0, ID_TC0, 0, 7); 
  TC_init(TC0, ID_TC1, 1, 8);
  TC_init(TC0, ID_TC2, 2, 11);
  TC_init(TC1, ID_TC3, 0, 17);
  
  /** Configura RTC */
  RTC_init();
  
  /* configura alarme do RTC */    
  rtc_set_date_alarm(RTC, 1, MOUNTH, 1, DAY);
  rtc_set_time_alarm(RTC, 1, HOUR, 1, MINUTE+1, 1, SECOND);
          
	while (1) {
		/* Entra em modo sleep */
		
	}
}