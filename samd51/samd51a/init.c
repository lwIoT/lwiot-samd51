/*
 * System initialization.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#include <sam.h>
#include <peripheral_clk_config.h>
#include <hal_init.h>
#include <core_cm4.h>

#include <lwiot/samd51/usb/hal_usb_device.h>
#include <hal_gpio.h>

#define PA24 GPIO(GPIO_PORTA, 24)
#define PA25 GPIO(GPIO_PORTA, 25)

void samd51_init()
{
	Adc *adcs[] = {ADC0, ADC1};

	/*if ( SysTick_Config( SystemCoreClock / 1000 ) ) {
		while ( true ) ;
	}*/

	NVIC_SetPriority (SysTick_IRQn,  (1 << __NVIC_PRIO_BITS) - 2);

	MCLK->APBAMASK.reg |= MCLK_APBAMASK_SERCOM0 | MCLK_APBAMASK_SERCOM1 | MCLK_APBAMASK_TC0 | MCLK_APBAMASK_TC1;
	MCLK->APBBMASK.reg |= MCLK_APBBMASK_SERCOM2 | MCLK_APBBMASK_SERCOM3 | MCLK_APBBMASK_TCC0 | MCLK_APBBMASK_TCC1 | MCLK_APBBMASK_TC3 | MCLK_APBBMASK_TC2;
	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC2 | MCLK_APBCMASK_TCC3 | MCLK_APBCMASK_TC4 | MCLK_APBCMASK_TC5;
	MCLK->APBDMASK.reg |= MCLK_APBDMASK_DAC | MCLK_APBDMASK_SERCOM4 | MCLK_APBDMASK_SERCOM5 | MCLK_APBDMASK_ADC0 | MCLK_APBDMASK_ADC1 | MCLK_APBDMASK_TCC4
	                      | MCLK_APBDMASK_TC6 | MCLK_APBDMASK_TC7 | MCLK_APBDMASK_SERCOM6 | MCLK_APBDMASK_SERCOM7;
	MCLK->APBAMASK.reg |= MCLK_APBAMASK_EIC;

	GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
	GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);


	for(int i=0; i<2; i++){

		adcs[i]->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV32_Val;
		adcs[i]->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;

		while( adcs[i]->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB );

		adcs[i]->SAMPCTRL.reg = 5;

		while( adcs[i]->SYNCBUSY.reg & ADC_SYNCBUSY_SAMPCTRL );

		adcs[i]->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;

		while( adcs[i]->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL );

		// Averaging (see datasheet table in AVGCTRL register description)
		adcs[i]->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 | ADC_AVGCTRL_ADJRES(0x0ul);

		while( adcs[i]->SYNCBUSY.reg & ADC_SYNCBUSY_AVGCTRL );
	}

	//analogReference( AR_DEFAULT ) ;

	GCLK->PCHCTRL[DAC_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK4_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
	while (GCLK->PCHCTRL[DAC_GCLK_ID].bit.CHEN == 0);

	while ( DAC->SYNCBUSY.bit.SWRST == 1 );
	DAC->CTRLA.bit.SWRST = 1;
	while ( DAC->SYNCBUSY.bit.SWRST == 1 );

	DAC->CTRLB.reg = DAC_CTRLB_REFSEL_VREFPU; // TODO: fix this once silicon bug is fixed

	DAC->DACCTRL[0].bit.REFRESH = 2;
	DAC->DACCTRL[1].bit.REFRESH = 2;
}

// Constants for Clock generators
#define GENERIC_CLOCK_GENERATOR_MAIN      (0u)

#if defined(__SAMD51__)
#define GENERIC_CLOCK_GENERATOR_XOSC32K   (3u)
#define GENERIC_CLOCK_GENERATOR_48M		  (1u)
#define GENERIC_CLOCK_GENERATOR_48M_SYNC	GCLK_SYNCBUSY_GENCTRL1
#define GENERIC_CLOCK_GENERATOR_100M	  (2u)
#define GENERIC_CLOCK_GENERATOR_100M_SYNC	GCLK_SYNCBUSY_GENCTRL2
#define GENERIC_CLOCK_GENERATOR_12M       (4u)
#define GENERIC_CLOCK_GENERATOR_12M_SYNC   GCLK_SYNCBUSY_GENCTRL4

//USE DPLL0 for 120MHZ
#define MAIN_CLOCK_SOURCE				  GCLK_GENCTRL_SRC_DPLL0

#define GENERIC_CLOCK_GENERATOR_1M		  (5u)
//#define CRYSTALLESS

#else

#define GENERIC_CLOCK_GENERATOR_XOSC32K   (1u)
#endif

static void sys_init( void )
{

//***************** SAMD51 ************************//
#if defined(__SAMD51__)
	NVMCTRL->CTRLA.reg |= NVMCTRL_CTRLA_RWS(0);

#ifndef CRYSTALLESS
	/* ----------------------------------------------------------------------------------------------
	 * 1) Enable XOSC32K clock (External on-board 32.768Hz oscillator)
	 */

	OSC32KCTRL->XOSC32K.reg = OSC32KCTRL_XOSC32K_ENABLE | OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_CGM_XT | OSC32KCTRL_XOSC32K_XTALEN;

	while( (OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY) == 0 ){
		/* Wait for oscillator to be ready */
	}

#endif //CRYSTALLESS

	//software reset

	GCLK->CTRLA.bit.SWRST = 1;
	while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_SWRST ){
		/* wait for reset to complete */
	}

#ifndef CRYSTALLESS
	/* ----------------------------------------------------------------------------------------------
	 * 2) Put XOSC32K as source of Generic Clock Generator 3
	 */
	GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_XOSC32K].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_XOSC32K) | //generic clock gen 3
	                                                     GCLK_GENCTRL_GENEN;
#else
	/* ----------------------------------------------------------------------------------------------
   * 2) Put OSCULP32K as source of Generic Clock Generator 3
   */
  GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_XOSC32K].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSCULP32K) | GCLK_GENCTRL_GENEN; //generic clock gen 3
#endif


	while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL3 ){
		/* Wait for synchronization */
	}

	/* ----------------------------------------------------------------------------------------------
	 * 3) Put OSCULP32K as source for Generic Clock Generator 0
	 */
	GCLK->GENCTRL[0].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSCULP32K) | GCLK_GENCTRL_GENEN;

	/* ----------------------------------------------------------------------------------------------
	 * 4) Enable DFLL48M clock
	 */

	while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL0 ){
		/* Wait for synchronization */
	}

	/* DFLL Configuration in Open Loop mode */

	OSCCTRL->DFLLCTRLA.reg = 0;
	//GCLK->PCHCTRL[OSCCTRL_GCLK_ID_DFLL48].reg = (1 << GCLK_PCHCTRL_CHEN_Pos) | GCLK_PCHCTRL_GEN(GCLK_PCHCTRL_GEN_GCLK3_Val);

	OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_CSTEP( 0x1 ) |
	                       OSCCTRL_DFLLMUL_FSTEP( 0x1 ) |
	                       OSCCTRL_DFLLMUL_MUL( 0 );

	while ( OSCCTRL->DFLLSYNC.reg & OSCCTRL_DFLLSYNC_DFLLMUL )
	{
		/* Wait for synchronization */
	}

	OSCCTRL->DFLLCTRLB.reg = 0;
	while ( OSCCTRL->DFLLSYNC.reg & OSCCTRL_DFLLSYNC_DFLLCTRLB )
	{
		/* Wait for synchronization */
	}

	OSCCTRL->DFLLCTRLA.reg |= OSCCTRL_DFLLCTRLA_ENABLE;
	while ( OSCCTRL->DFLLSYNC.reg & OSCCTRL_DFLLSYNC_ENABLE )
	{
		/* Wait for synchronization */
	}

	OSCCTRL->DFLLVAL.reg = OSCCTRL->DFLLVAL.reg;
	while( OSCCTRL->DFLLSYNC.bit.DFLLVAL );

	OSCCTRL->DFLLCTRLB.reg = OSCCTRL_DFLLCTRLB_WAITLOCK |
	                         OSCCTRL_DFLLCTRLB_CCDIS | OSCCTRL_DFLLCTRLB_USBCRM ;

	while ( !OSCCTRL->STATUS.bit.DFLLRDY )
	{
		/* Wait for synchronization */
	}

	GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_1M].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL_Val) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_DIV(48u);

	while ( GCLK->SYNCBUSY.bit.GENCTRL5 ){
		/* Wait for synchronization */
	}


	/* ------------------------------------------------------------------------
	* Set up the PLLs
	*/

	//PLL0 is 120MHz
	GCLK->PCHCTRL[OSCCTRL_GCLK_ID_FDPLL0].reg = (1 << GCLK_PCHCTRL_CHEN_Pos) | GCLK_PCHCTRL_GEN(GCLK_PCHCTRL_GEN_GCLK5_Val);

	// This rounds to nearest full-MHz increment; not currently using frac
	OSCCTRL->Dpll[0].DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(0x00) | OSCCTRL_DPLLRATIO_LDR((F_CPU - 500000) / 1000000);

	while(OSCCTRL->Dpll[0].DPLLSYNCBUSY.bit.DPLLRATIO);

	//MUST USE LBYPASS DUE TO BUG IN REV A OF SAMD51
	OSCCTRL->Dpll[0].DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK_GCLK | OSCCTRL_DPLLCTRLB_LBYPASS;

	OSCCTRL->Dpll[0].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;

	while( OSCCTRL->Dpll[0].DPLLSTATUS.bit.CLKRDY == 0 || OSCCTRL->Dpll[0].DPLLSTATUS.bit.LOCK == 0 );

	//PLL1 is 100MHz
	GCLK->PCHCTRL[OSCCTRL_GCLK_ID_FDPLL1].reg = (1 << GCLK_PCHCTRL_CHEN_Pos) | GCLK_PCHCTRL_GEN(GCLK_PCHCTRL_GEN_GCLK5_Val);

	OSCCTRL->Dpll[1].DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(0x00) | OSCCTRL_DPLLRATIO_LDR(99); //100 Mhz

	while(OSCCTRL->Dpll[1].DPLLSYNCBUSY.bit.DPLLRATIO);

	//MUST USE LBYPASS DUE TO BUG IN REV A OF SAMD51
	OSCCTRL->Dpll[1].DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK_GCLK | OSCCTRL_DPLLCTRLB_LBYPASS;

	OSCCTRL->Dpll[1].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;

	while( OSCCTRL->Dpll[1].DPLLSTATUS.bit.CLKRDY == 0 || OSCCTRL->Dpll[1].DPLLSTATUS.bit.LOCK == 0 );


	/* ------------------------------------------------------------------------
	* Set up the peripheral clocks
	*/

	//48MHZ CLOCK FOR USB AND STUFF
	GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_48M].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL_Val) |
	                                                 GCLK_GENCTRL_IDC |
	                                                 //GCLK_GENCTRL_OE |
	                                                 GCLK_GENCTRL_GENEN;

	while ( GCLK->SYNCBUSY.reg & GENERIC_CLOCK_GENERATOR_48M_SYNC)
	{
		/* Wait for synchronization */
	}

	//100MHZ CLOCK FOR OTHER PERIPHERALS
	GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_100M].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DPLL1_Val) |
	                                                  GCLK_GENCTRL_IDC |
	                                                  //GCLK_GENCTRL_OE |
	                                                  GCLK_GENCTRL_GENEN;

	while ( GCLK->SYNCBUSY.reg & GENERIC_CLOCK_GENERATOR_100M_SYNC)
	{
		/* Wait for synchronization */
	}

	//12MHZ CLOCK FOR DAC
	GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_12M].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL_Val) |
	                                                 GCLK_GENCTRL_IDC |
	                                                 GCLK_GENCTRL_DIV(4) |
	                                                 //GCLK_GENCTRL_DIVSEL |
	                                                 //GCLK_GENCTRL_OE |
	                                                 GCLK_GENCTRL_GENEN;

	while ( GCLK->SYNCBUSY.reg & GENERIC_CLOCK_GENERATOR_12M_SYNC)
	{
		/* Wait for synchronization */
	}

	/*---------------------------------------------------------------------
	 * Set up main clock
	 */

	GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = GCLK_GENCTRL_SRC(MAIN_CLOCK_SOURCE) |
	                                                  GCLK_GENCTRL_IDC |
	                                                  //GCLK_GENCTRL_OE |
	                                                  GCLK_GENCTRL_GENEN;


	while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL0 )
	{
		/* Wait for synchronization */
	}

	MCLK->CPUDIV.reg = MCLK_CPUDIV_DIV_DIV1;

	/* Use the LDO regulator by default */
	SUPC->VREG.bit.SEL = 0;


	/* If desired, enable cache! */
#if defined(ENABLE_CACHE)
	__disable_irq();
  CMCC->CTRL.reg = 1;
  __enable_irq();
#endif

	/*---------------------------------------------------------------------
	 * Start up the "Debug Watchpoint and Trace" unit, so that we can use
	 * it's 32bit cycle counter for timing.
	 */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/* ----------------------------------------------------------------------------------------------
	 * 5) Load AC factory calibration values
	 */

	uint32_t bias0 = (*((uint32_t *)AC_FUSES_BIAS0_ADDR) & AC_FUSES_BIAS0_Msk) >> AC_FUSES_BIAS0_Pos;
	AC->CALIB.reg = AC_CALIB_BIAS0(bias0);

	/* ----------------------------------------------------------------------------------------------
	 * 6) Load ADC factory calibration values
	 */

	// ADC0 Bias Calibration
	uint32_t biascomp = (*((uint32_t *)ADC0_FUSES_BIASCOMP_ADDR) & ADC0_FUSES_BIASCOMP_Msk) >> ADC0_FUSES_BIASCOMP_Pos;
	uint32_t biasr2r = (*((uint32_t *)ADC0_FUSES_BIASR2R_ADDR) & ADC0_FUSES_BIASR2R_Msk) >> ADC0_FUSES_BIASR2R_Pos;
	uint32_t biasref = (*((uint32_t *)ADC0_FUSES_BIASREFBUF_ADDR) & ADC0_FUSES_BIASREFBUF_Msk) >> ADC0_FUSES_BIASREFBUF_Pos;

	ADC0->CALIB.reg = ADC_CALIB_BIASREFBUF(biasref)
	                  | ADC_CALIB_BIASR2R(biasr2r)
	                  | ADC_CALIB_BIASCOMP(biascomp);

	// ADC1 Bias Calibration
	biascomp = (*((uint32_t *)ADC1_FUSES_BIASCOMP_ADDR) & ADC1_FUSES_BIASCOMP_Msk) >> ADC1_FUSES_BIASCOMP_Pos;
	biasr2r = (*((uint32_t *)ADC1_FUSES_BIASR2R_ADDR) & ADC1_FUSES_BIASR2R_Msk) >> ADC1_FUSES_BIASR2R_Pos;
	biasref = (*((uint32_t *)ADC1_FUSES_BIASREFBUF_ADDR) & ADC1_FUSES_BIASREFBUF_Msk) >> ADC1_FUSES_BIASREFBUF_Pos;

	ADC1->CALIB.reg = ADC_CALIB_BIASREFBUF(biasref)
	                  | ADC_CALIB_BIASR2R(biasr2r)
	                  | ADC_CALIB_BIASCOMP(biascomp);

	/* ----------------------------------------------------------------------------------------------
	 * 7) Load USB factory calibration values
	 */

	//USB Calibration
	uint32_t usbtransn = (*((uint32_t *)USB_FUSES_TRANSN_ADDR) & USB_FUSES_TRANSN_Msk) >> USB_FUSES_TRANSN_Pos;
	uint32_t usbtransp = (*((uint32_t *)USB_FUSES_TRANSP_ADDR) & USB_FUSES_TRANSP_Msk) >> USB_FUSES_TRANSP_Pos;
	uint32_t usbtrim = (*((uint32_t *)USB_FUSES_TRIM_ADDR) & USB_FUSES_TRIM_Msk) >> USB_FUSES_TRIM_Pos;
	USB->DEVICE.PADCAL.reg = USB_PADCAL_TRIM(usbtrim)
	                         | USB_PADCAL_TRANSN(usbtransn)
	                         | USB_PADCAL_TRANSP(usbtransp);

#endif
}

/* The USB module requires a GCLK_USB of 48 MHz ~ 0.25% clock
 * for low speed and full speed operation. */
#if (CONF_GCLK_USB_FREQUENCY > (48000000 + 48000000 / 400)) || (CONF_GCLK_USB_FREQUENCY < (48000000 - 48000000 / 400))
#warning USB clock should be 48MHz ~ 0.25% clock, check your configuration!
#endif

static void USB_DEVICE_INSTANCE_PORT_init(void)
{
	gpio_set_pin_direction(PA24, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PA24, false);
	gpio_set_pin_pull_mode(PA24, GPIO_PULL_OFF);
	gpio_set_pin_function(PA24,	PINMUX_PA24H_USB_DM);
	gpio_set_pin_direction(PA25, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PA25, false);
	gpio_set_pin_pull_mode(PA25, GPIO_PULL_OFF);
	gpio_set_pin_function(PA25,	PINMUX_PA25H_USB_DP);
}

static void USB_DEVICE_INSTANCE_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, USB_GCLK_ID, CONF_GCLK_USB_SRC | GCLK_PCHCTRL_CHEN);
	hri_mclk_set_AHBMASK_USB_bit(MCLK);
	hri_mclk_set_APBBMASK_USB_bit(MCLK);
}

static void USB_DEVICE_INSTANCE_init(void)
{
	USB_DEVICE_INSTANCE_CLOCK_init();
	usb_d_init();
	USB_DEVICE_INSTANCE_PORT_init();
}

void chip_init()
{
	SystemInit();
	sys_init();
	samd51_init();

	USB_DEVICE_INSTANCE_init();
}
