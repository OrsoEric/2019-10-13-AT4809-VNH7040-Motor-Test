/****************************************************************************
**	INCLUDES
****************************************************************************/

#include "global.h"

/****************************************************************************
**	MACROS
****************************************************************************/

//Configure a protected register in the initialization phase
//CPP: Writing in this register allows operation on IO protected registers for the next 4CLK
#define INIT_CONFIG_PROTECTED( target_register, value_mask, value_position, value )	\
	CCP = CCP_IOREG_gc, (target_register) = ( ((target_register) & ~(value_mask)) | (((value) << (value_position)) & (value_mask))  )


/****************************************************************************
**	FUNCTIONS PROTOTYPES
****************************************************************************/

//Initialize the micro controller fuses
extern void init_fuses( void );
//Initialize clock systems
extern void init_clock( void );
//Initialize pin map
extern void init_pin( void );
//Initialize RTC timer as periodic interrupt
extern void init_rtc( void );
//Initialize timer type A. AT4809 has a single of such timers.
extern void init_timer0a_split( void );

/****************************************************************************
**	FUNCTIONS DECLARATIONS
****************************************************************************/

/****************************************************************************
**  Function
**  init |
****************************************************************************/
//! @return bool |
//! @brief main init function
//! @details initialize all embedded peripherals and pins.
//!	remember to call the initialization functions in the main init
/***************************************************************************/

void init(void)
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Initialize clock systems
	init_clock();
	//initialize pin configuration
	init_pin();
	//Initialize RTC timer as Periodic interrupt source: RTC_PIT_vect
	init_rtc();
	//Initialize timer type A
	init_timer0a_split();
	
	//Activate interrupts
	sei();
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End: init

/****************************************************************************
**  Function
**  init_clock |
****************************************************************************/
//! @brief initialize clock systems
//! @details setup the clock system multiplexers and the clock output
/***************************************************************************/

void init_clock( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Main clock switch
	INIT_CONFIG_PROTECTED( CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_gm, CLKCTRL_CLKSEL_gp, (CLKCTRL_CLKSEL_t)CLKCTRL_CLKSEL_OSC20M_gc);
	//Configure CLK_OUT pin as disabled
	INIT_CONFIG_PROTECTED( CLKCTRL.MCLKCTRLA, CLKCTRL_CLKOUT_bm, CLKCTRL_CLKOUT_bp, 0);
	//Disable the main clock prescaler
	INIT_CONFIG_PROTECTED( CLKCTRL.MCLKCTRLB, CLKCTRL_PEN_bm, CLKCTRL_PEN_bm, 0);
	//Set the main clock prescaler to 2
	INIT_CONFIG_PROTECTED( CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_gm, CLKCTRL_PDIV_gp, (CLKCTRL_PDIV_t)CLKCTRL_PDIV_2X_gc);
	//Disable the clock multiplexer and prescaler protection
	INIT_CONFIG_PROTECTED( CLKCTRL.MCLKLOCK, CLKCTRL_LOCKEN_bm, CLKCTRL_LOCKEN_bp, 0);
	
	CLKCTRL.OSC20MCTRLA |= CLKCTRL_RUNSTDBY_bm; //1<<1;
	CCP = CCP_IOREG_gc;
	CLKCTRL.OSC20MCALIBB |= CLKCTRL_LOCK_bm; //1<<7
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End: init_clock

/****************************************************************************
**  Function
**  init_pin |
****************************************************************************/
//! @brief initialize pin configuration
//! @details Initialize pin configuration and multiplexers
/***************************************************************************/

void init_pin( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//
	
	//----------------------------------------------------------------
	//!	PORTA
	//!	PA0				:
	//!	PA1				:
	//!	PA2				: 
	//!	PA3				: 
	//!	PA4				: 
	//!	PA5				: 
	//!	PA6				:
	//!	PA7				:
	//----------------------------------------------------------------
	//				0		1		2		3		4		5		6		7
	PORT_A_CONFIG(	PIN_L,	PIN_L,	PIN_L,	PIN_L,	PIN_L,	PIN_L,	PIN_Z,	PIN_Z );
	
	//----------------------------------------------------------------
	//!	PORTB
	//!	PB0				:
	//!	PB1				:
	//!	PB2				: DRV0_PWM
	//!	PB3				: DRV1_PWM
	//!	PB4				: DRV2_PWM
	//!	PB5				: DRV3_PWM
	//!	PB6				:
	//!	PB7				:
	//----------------------------------------------------------------
	//				0		1		2		3		4		5		6		7
	PORT_B_CONFIG(	PIN_Z,	PIN_Z,	PIN_L,	PIN_L,	PIN_L,	PIN_L,	PIN_Z,	PIN_Z );
	
	//----------------------------------------------------------------
	//!	PORTC
	//!	PC0				:
	//!	PC1				:
	//!	PC2				: DRV0_CTRLA
	//!	PC3				: DRV0_CTRLB
	//!	PC4				:
	//!	PC5				:
	//!	PC6				:
	//!	PC7				:
	//----------------------------------------------------------------
	//				0		1		2		3		4		5		6		7
	PORT_C_CONFIG(	PIN_Z,	PIN_Z,	PIN_L,	PIN_L,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z );
	
	//----------------------------------------------------------------
	//!	PORTD
	//!	PD0				:
	//!	PD1				:
	//!	PD2				:
	//!	PD3				:
	//!	PD4				:
	//!	PD5				:
	//!	PD6				:
	//!	PD7				:
	//----------------------------------------------------------------
	//				0		1		2		3		4		5		6		7
	PORT_D_CONFIG(	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z );
	
	//----------------------------------------------------------------
	//!	PORTE
	//!	PE0				:
	//!	PE1				:
	//!	PE2				:
	//!	PE3				:
	//!	PE4				:
	//!	PE5				:
	//!	PE6				:
	//!	PE7				:
	//----------------------------------------------------------------
	//				0		1		2		3		4		5		6		7
	PORT_E_CONFIG(	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z );
	
	//----------------------------------------------------------------
	//!	PORTF
	//!	PF0				:
	//!	PF1				:
	//!	PF2				:
	//!	PF3				:
	//!	PF4				: DRV_SEN Driver SENSE Enable
	//!	PF5				: DRV_DIAG Driver Diagnostic enable | Curiosity Nano LED
	//!	PF6				:
	//!	PF7				:
	//----------------------------------------------------------------
	//				0		1		2		3		4		5		6		7
	PORT_F_CONFIG(	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_L,	PIN_L,	PIN_Z,	PIN_Z );	
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End: init_pin

/****************************************************************************
**  Function
**  init_rtc |
****************************************************************************/
//! @brief Initialize RTC timer as periodic interrupt
//! @details 
//! Interrupt vectors:
//!		RTC_CNT_vect
//!		RTC_PIT_vect
/***************************************************************************/

void init_rtc( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//! Fetch registers
	uint8_t ctrla_tmp		= RTC.CTRLA;
	uint8_t intctrl_tmp		= RTC.INTCTRL;
	uint8_t dgbctrl_tmp		= RTC.DBGCTRL;
	uint8_t clksel_tmp		= RTC.CLKSEL;
	uint8_t pitctrla_tmp	= RTC.PITCTRLA;
	uint8_t pitintctrl_tmp	= RTC.PITINTCTRL;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//! Enable the RTC timer
	SET_BIT( ctrla_tmp, RTC_RTCEN_bp );
	
	//! Enable the RTC Correction
	//SET_BIT( ctrla_tmp, 2 );
	
	//! Let the RTC timer run in standby mode
	SET_BIT( ctrla_tmp, RTC_RUNSTDBY_bp );
	
	//! Let the RTC run in debug when CPU is halted
	//SET_BIT( dgbctrl_tmp, RTC_DBGRUN_bp );
	
	//----------------------------------------------------------------
	//! RTC Clock Source
	//----------------------------------------------------------------
	//	Clock source for the RTC timer. Select only one
	
	SET_MASKED_BIT( clksel_tmp, RTC_CLKSEL_gm, RTC_CLKSEL_INT32K_gc );
	//SET_MASKED_BIT( clksel_tmp, RTC_CLKSEL_gm, RTC_CLKSEL_INT1K_gc );
	//SET_MASKED_BIT( clksel_tmp, RTC_CLKSEL_gm, RTC_CLKSEL_TOSC32K_gc );
	//SET_MASKED_BIT( clksel_tmp, RTC_CLKSEL_gm, RTC_CLKSEL_EXTCLK_gc );
	
	//----------------------------------------------------------------
	//! RTC Clock Prescaler
	//----------------------------------------------------------------
	//	Set prescaler. Only activate one
	
	SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV1_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV2_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV4_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV8_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV16_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV32_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV64_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV128_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV256_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV512_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV1024_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV2048_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV4096_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV8192_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV16384_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV32768_gc );
	
	//----------------------------------------------------------------
	//! RTC Periodic Interrupt period
	//----------------------------------------------------------------
	
	//! Enable Periodic Interrupt timer
	SET_BIT( pitctrla_tmp, RTC_PITEN_bp );
	
	//! Period for the periodic interrupt. Activate only one
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_OFF_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC4_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC8_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC16_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC32_gc );
	SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC64_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC128_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC256_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC512_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC1024_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC2048_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC4096_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC8192_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC16384_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC32768_gc );
	
	//----------------------------------------------------------------
	//! RTC Interrupts
	//----------------------------------------------------------------
	
	//! Enable overflow interrupt
	//SET_BIT( intctrl_tmp, RTC_OVF_bp );
	//! Enable Compare Match interrupt
	//SET_BIT( intctrl_tmp, RTC_CMP_bp );
	//! Enable Periodic Interrupt timer
	SET_BIT( pitintctrl_tmp, RTC_PI_bp );
	

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	//Calibration PPM of the RTC counter. Meant to be done in software against more precise clock sources.
	RTC.CALIB = (uint8_t)0x00;
	
	//Wait for the ***
	//while (IS_BIT_ONE(RTC.STATUS, RTC_PERBUSY_bp));
	RTC.PER = (uint16_t)0;
	//Compare register for compare interrupt
	RTC.CMP = (uint16_t)0;
	
	//! Registers write back
	//Configuration registers
	RTC.DBGCTRL = dgbctrl_tmp;
	RTC.CLKSEL = clksel_tmp;
	RTC.PITCTRLA = pitctrla_tmp;
	//Write this register last as it activates the timer
	RTC.CTRLA = ctrla_tmp;
	//Activate interrupts
	RTC.INTCTRL = intctrl_tmp;
	RTC.PITINTCTRL = pitintctrl_tmp;
	
	return;
}	//End: init_rtc

/****************************************************************************
**  Function
**  init_timer_a |
****************************************************************************/
//! @brief initialize timer type a in split mode as two 8bit timers with six compare channels
//! @details setup the only timer type A of the AT4809
//!	
//!	Clock from event control is disabled in SPLIT mode
//!	
//! Interrupt vectors available:
//! TCA0_LUNF_vect 
//! TCA0_OVF_vect
//! TCA0_HUNF_vect 
//! TCA0_LCMP0_vect 
//! TCA0_CMP0_vect 
//! TCA0_CMP1_vect  
//! TCA0_LCMP1_vect 
//! TCA0_LCMP2_vect 
//! TCA0_CMP2_vect
/***************************************************************************/

void init_timer0a_split( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Load temporary registers
	uint8_t ctrla_tmp			= TCA0.SPLIT.CTRLA;
	uint8_t ctrlb_tmp			= TCA0.SPLIT.CTRLB;
	uint8_t ctrlc_tmp			= TCA0.SPLIT.CTRLC;
	uint8_t ctrld_tmp			= TCA0.SPLIT.CTRLD;
	uint8_t ctrle_tmp			= TCA0.SPLIT.CTRLESET;
	uint8_t dbgctrl_tmp			= TCA0.SPLIT.DBGCTRL;
	uint8_t port_mux_tca0_tmp	= PORTMUX.TCAROUTEA;
	uint8_t intctrl_tmp			= TCA0.SPLIT.INTCTRL;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

		//----------------------------------------------------------------
		//! Enable Split Mode
		//----------------------------------------------------------------
		//	Function of registers change according to the mode.
		//	0 = 3x 16bit
		//	1 = 6x 8bit

	SET_BIT( ctrld_tmp, TCA_SPLIT_SPLITM_bp );

		//----------------------------------------------------------------
		//! Enable TCA
		//----------------------------------------------------------------
		//	0 = disabled
		//	1 = enabled
	
	SET_BIT( ctrla_tmp, TCA_SPLIT_ENABLE_bp );
	
		//----------------------------------------------------------------
		//! TCA Clock Prescaler
		//----------------------------------------------------------------
		//	Set the clock prescaler of this TCA. Activate only one value
		
	//SET_MASKED_BIT( ctrla_tmp, TCA_SPLIT_CLKSEL_gm , TCA_SPLIT_CLKSEL_DIV1_gc );
	//SET_MASKED_BIT( ctrla_tmp, TCA_SPLIT_CLKSEL_gm , TCA_SPLIT_CLKSEL_DIV2_gc );
	SET_MASKED_BIT( ctrla_tmp, TCA_SPLIT_CLKSEL_gm , TCA_SPLIT_CLKSEL_DIV4_gc );
	//SET_MASKED_BIT( ctrla_tmp, TCA_SPLIT_CLKSEL_gm , TCA_SPLIT_CLKSEL_DIV8_gc );
	//SET_MASKED_BIT( ctrla_tmp, TCA_SPLIT_CLKSEL_gm , TCA_SPLIT_CLKSEL_DIV16_gc );
	//SET_MASKED_BIT( ctrla_tmp, TCA_SPLIT_CLKSEL_gm , TCA_SPLIT_CLKSEL_DIV64_gc );
	//SET_MASKED_BIT( ctrla_tmp, TCA_SPLIT_CLKSEL_gm , TCA_SPLIT_CLKSEL_DIV256_gc );
	//SET_MASKED_BIT( ctrla_tmp, TCA_SPLIT_CLKSEL_gm , TCA_SPLIT_CLKSEL_DIV1024_gc );

		//----------------------------------------------------------------
		//! TCA Enable compare output and waveform output pin override for each compare channel
		//----------------------------------------------------------------

	SET_BIT( ctrlb_tmp, TCA_SPLIT_LCMP0EN_bp );
	SET_BIT( ctrlb_tmp, TCA_SPLIT_LCMP1EN_bp );
	SET_BIT( ctrlb_tmp, TCA_SPLIT_LCMP2EN_bp );
	SET_BIT( ctrlb_tmp, TCA_SPLIT_HCMP0EN_bp );
	SET_BIT( ctrlb_tmp, TCA_SPLIT_HCMP1EN_bp );
	SET_BIT( ctrlb_tmp, TCA_SPLIT_HCMP2EN_bp );

		//----------------------------------------------------------------
		//! TCA Enable timer commands
		//----------------------------------------------------------------

	//Disable force commands
	SET_MASKED_BIT( ctrle_tmp, (uint8_t)0x03 , (uint8_t)0x00 );
	//ENABLE force commands for both channels
	//SET_MASKED_BIT( ctrle_tmp, (uint8_t)0x03 , (uint8_t)0x03 );

		//----------------------------------------------------------------
		//! ENABLE TCA interrupts
		//----------------------------------------------------------------

	//Underflow of low counter
	//SET_BIT( intctrl_tmp, TCA_SPLIT_LUNF_bp );
	//Underflow of high counter
	//SET_BIT( intctrl_tmp, TCA_SPLIT_HUNF_bp );
	//Compare channel
	//SET_BIT( intctrl_tmp, TCA_SPLIT_LCMP0_bp );
	//SET_BIT( intctrl_tmp, TCA_SPLIT_LCMP1_bp );
	//SET_BIT( intctrl_tmp, TCA_SPLIT_LCMP2_bp );

		//----------------------------------------------------------------
		//! ENABLE TCA debug
		//----------------------------------------------------------------
	
	SET_BIT( dbgctrl_tmp, TCA_SPLIT_DBGRUN_bp );

		//----------------------------------------------------------------
		//! Set TCA waveform outputs to a given port. You can activate only one port
		//----------------------------------------------------------------
		
	//SET_MASKED_BIT( port_mux_tca0_tmp, PORTMUX_TCA0_gm, PORTMUX_TCA0_PORTA_gc );	//default
	SET_MASKED_BIT( port_mux_tca0_tmp, PORTMUX_TCA0_gm, PORTMUX_TCA0_PORTB_gc );
	//SET_MASKED_BIT( port_mux_tca0_tmp, PORTMUX_TCA0_gm, PORTMUX_TCA0_PORTC_gc );
	//SET_MASKED_BIT( port_mux_tca0_tmp, PORTMUX_TCA0_gm, PORTMUX_TCA0_PORTD_gc );
	//SET_MASKED_BIT( port_mux_tca0_tmp, PORTMUX_TCA0_gm, PORTMUX_TCA0_PORTE_gc );
	//SET_MASKED_BIT( port_mux_tca0_tmp, PORTMUX_TCA0_gm, PORTMUX_TCA0_PORTF_gc );
		
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	//! Register write back.
	//Write back control registers
	TCA0.SPLIT.CTRLB = ctrlb_tmp;
	TCA0.SPLIT.CTRLC = ctrlc_tmp;
	TCA0.SPLIT.CTRLD = ctrld_tmp;
	TCA0.SPLIT.CTRLESET = ctrle_tmp;
	TCA0.SPLIT.DBGCTRL = dbgctrl_tmp;
	//Write back output waveform port selector
	PORTMUX.TCAROUTEA = port_mux_tca0_tmp;
	
	//Period registers for the High and Low 8bit counters
	TCA0.SPLIT.LPER = (uint8_t)255;
	TCA0.SPLIT.HPER = (uint8_t)255;
	//Initial PWM settings for channels L0 to L2
	TCA0.SPLIT.LCMP0 = (uint8_t)0x00;
	TCA0.SPLIT.LCMP1 = (uint8_t)0x00;
	TCA0.SPLIT.LCMP2 = (uint8_t)0x00;
	//Initial PWM settings for channels H0 to H2
	TCA0.SPLIT.HCMP0 = (uint8_t)0x00;
	TCA0.SPLIT.HCMP1 = (uint8_t)0x00;
	TCA0.SPLIT.HCMP2 = (uint8_t)0x0;	
	
	//Write back control A for last as it's the one that sets the clock and starts the timer
	TCA0.SPLIT.CTRLA = ctrla_tmp;
	//Write back interrupt enable
	TCA0.SPLIT.INTCTRL = intctrl_tmp;

	return;
}	//End: init_timer0a