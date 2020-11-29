//////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Nathan Kaltenbrun
// ECE 353, 01-Code-IO-Pins, 09/09/2020
//
//////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ece353.h"

/********************************************************************
 * Initialize hardware resources used to control LED1
 *********************************************************************/
void ece353_led1_init(void){
    // Set Port 1.0 as an output
    P1->DIR |= BIT0;
    
    // Turn off LED1
    P1->OUT &= ~BIT0;
}

/*********************************************************************
 * Initialize hardware resources used to control Button1
 *********************************************************************/
void ece353_button1_init(void){
    // Set Port 1.1 as an input
    P1->DIR &= ~BIT1;
    
    // Set Port 1.1 with a Resistor
    P1->REN |= BIT1;
    
    // Select Pull-Up Resistor
    P1->OUT |= BIT1;
}

/*********************************************************************
 * Turn LED1 ON/OFF
 * 
 * Parameters
 * on:  if TRUE, turn LED1 On
 *      if FLASE, turn LED1 Off
 *********************************************************************/
void ece353_led1(bool on){
    if(on){
        // Turn LED1 On
        P1->OUT |= BIT0;
    }
    else{
        // Turn LED1 Off
        P1->OUT &= ~BIT0;
    }
}


/*********************************************************************
 * Returns if Button1 is currently pressed.
 * 
 * Parameters
 * 
 * Returns
 *  TRUE: Button1 is pressed
 *  FALSE: Button1 is not pressed
 *********************************************************************/
bool ece353_button1(void){
    if((P1->IN & BIT1) == 0){ // Active Low Button
        // Button is pressed
        return true;
    }
    else{
        // Button is not pressed
        return false;
    }
}


/*****************************************************
* Initialize hardware resources used to control RGBLED
*****************************************************/
void ece353_rgb_init(void){
    // Initilize RGB LED as Outputs
    P2->DIR |= BIT0; // Red
    P2->DIR |= BIT1; // Green
    P2->DIR |= BIT2; // Blue

    // Turn RGB LED OFF
    P2->OUT &= ~BIT0;
    P2->OUT &= ~BIT1;
    P2->OUT &= ~BIT2;
}

/*****************************************************
* Turn RGBLED ON/Off.
*
* Parameters
*  red_on      :   if true,  turn RGBLED.RED on
*                  if false, turn RGBLED.RED off
*  green_on    :   if true,  turn RGBLED.GREEN on
*                  if false, turn RGBLED.GREEN off
*  blue_on     :   if true,  turn RGBLED.BLUE on
*                  if false, turn RGBLED.BLUE off
*****************************************************/
void ece353_rgb(bool red_on, bool green_on, bool blue_on){
    // Turn RGB LED colors on by param
    // Red
    if(red_on){
        P2->OUT |= BIT0;
    }
    else{
        P2->OUT &= ~BIT0;
    }

    // Green
    if(green_on){
            P2->OUT |= BIT1;
    }
    else{
            P2->OUT &= ~BIT1;
    }

    // Blue
    if(blue_on){
            P2->OUT |= BIT2;
    }
    else{
            P2->OUT &= ~BIT2;
    }
}

/*****************************************************
* Initialize hardware resources used to control Button2
*****************************************************/
void ece353_button2_init(void){
    // Set Port 1.4 as an input
    P1->DIR &= ~BIT4;

    // Set Port 1.4 with a Resistor
    P1->REN |= BIT4;

    // Select Pull-Up Resistor
    P1->OUT |= BIT4;
}

/*****************************************************
* Returns if Button2 is currently pressed.
*
* Parameters
*
* Returns
*      true    :   Button2 is pressed
*      false   :   Button2 is NOT pressed
*****************************************************/
bool ece353_button2(void){
    if((P1->IN & BIT4) == 0){ // Active Low Button
        // Button is pressed
        return true;
    }
    else{
        // Button is not pressed
        return false;
    }
}


/*****************************************************
 * Busy waits for a given number of clock cycles
 *
 * Parameters
 *      ticks : Number of ticks to wait
 *
 * Returns
 *      None
 *****************************************************/
void ece353_T32_1_wait(uint32_t ticks){
    // Stop the timer
    TIMER32_1->CONTROL = 0; // Turns whole register to zero

    // Set the timer to be a 32-bit, one-shot
    TIMER32_1->CONTROL = TIMER32_CONTROL_ONESHOT | TIMER32_CONTROL_SIZE;

    // Set the load register
    TIMER32_1->LOAD = ticks;

    // Start the timer
    TIMER32_1->CONTROL |= TIMER32_CONTROL_ENABLE;

    // Wait until it reaches 0
    while(TIMER32_1->VALUE != 0){
        // Busy wait
    }
}

    /*****************************************************
    * Busy waits for 100mS and then returns.
    *
    * Timer32_1 MUST be configured as a 16-bit timer.
    * Assume that the MCU clock runs at 3MHz.  You will
    * need to use a pre-scalar in order to achieve a delay
    * of 100mS.
    *
    * Parameters
    *      None
    * Returns
    *      None
    *****************************************************/
    void ece353_T32_1_wait_100mS(void){
        // Stop the timer
        TIMER32_1->CONTROL = 0; // Turns whole register to zero

        // Set the timer to be a 16-bit, one-shot, pre-scalar to 16
        TIMER32_1->CONTROL = TIMER32_CONTROL_ONESHOT | TIMER32_CONTROL_PRESCALE_1;

        // Set the load register
        TIMER32_1->LOAD = 0x493E; // 18750

        // Start the timer
        TIMER32_1->CONTROL |= TIMER32_CONTROL_ENABLE;

        // Wait until it reaches 0
        while(TIMER32_1->VALUE != 0){
            // Busy wait
        }
    }

    /*****************************************************
    * Debounces Button1 using Timer32_1.
    * This function does not return until Button 1 has
    * been pressed for a minimum of 5 seconds.
    *
    * Waiting 5 Seconds will require you to call
    * ece353_T32_1_wait_100mS multiple times.
    *
    * Parameters
    *      None
    * Returns
    *      None
    *****************************************************/
    void ece353_button1_wait_for_press(void){
        // Counts the amount of continuous pressing
        uint16_t counter = 0;

        while(counter < 50){ // For 5 seconds
            if(ece353_button1()){ // If button is still pressed
                ece353_T32_1_wait_100mS(); // Wait 100ms to check again
                counter++; // Increment amount of times waited
            }
            else{ // Button was not pressed
                counter = 0; // Restart the counter
            }
        }
    }



    /****************************************************
     * Initialize hardware resources used to control S1
     * on the MKII
     ****************************************************/
    void ece353_MKII_S1_Init(void){
        // Configure as input
        P5->DIR &= ~BIT1;
    }

    /****************************************************
     * Initialize hardware resources used to control S2
     * on the MKII
     ****************************************************/
    void ece353_MKII_S2_Init(void){
        // Configure as input
        P3->DIR &= ~BIT5;
    }

    /****************************************************
     * Returns if MKII.S1 is currently pressed
     *
     * Parameters
     *
     * Returns
     *      true : Button1 is pressed
     *      false : Button1 is NOT pressed
     ****************************************************/
    bool ece353_MKII_S1(void){
        if((P5->IN & BIT1) == 0){ // Active Low Button
            // Button is pressed
            return true;
        }
        else{
            // Button is not pressed
            return false;
        }
    }

    /****************************************************
     * Returns if MKII.S2 is currently pressed
     *
     * Parameters
     *
     * Returns
     *      true : Button2 is pressed
     *      false : Button2 is NOT pressed
     ****************************************************/
    bool ece353_MKII_S2(void){
        if((P3->IN & BIT5) == 0){ // Active Low Button
            // Button is pressed
            return true;
        }
        else{
            // Button is not pressed
            return false;
        }
    }

    /****************************************************
     * Sets the PWM period of the Buzzer. The duty cycle
     * will be set to 50%
     *
     * Parameters
     *      tick_period : Period of PWM Pulse
     *
     * Returns
     *      none
     ***************************************************/
    void ece353_MKII_Buzzer_Init(uint16_t ticks_period){
        // Set P2.7 to be a GPIO OUTPUT Pin
        P2->DIR |= BIT7;

        // the TimerA PWM Controller will control the buzzer on the MKII
        // P2.7 <--> TA0.4
        P2->SEL0 |= BIT7;
        P2->SEL1 &= ~BIT7;

        // Turn OFF TA0
        TIMER_A0->CTL = 0;

        // Set the period of the timer.
        TIMER_A0->CCR[0] = ticks_period - 1; // Zero index

        // Configure Buzzer Duty Cycle to 50%
        TIMER_A0->CCR[4] = (ticks_period / 2) - 1;

        // Configure TA0.4 for RESET/SET Mode
        TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7;

        // Select the master clock as the timer source
        TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK;
    }

    /***************************************************
     * Turns the Buzzer on
     *
     * Parameters
     *      None
     * Returns
     *      None
     ***************************************************/
    void ece353_MKII_Buzzer_On(void){
        // Clear the current Mode Control Bits
        TIMER_A0->CTL &= ~TIMER_A_CTL_MC_MASK;

        // Set Mode Control to UP and Clear the current count
        TIMER_A0->CTL |= TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR;
    }

    /***************************************************
     * Turns the Buzzer off
     *
     * Parameters
     *      None
     * Returns
     *      None
     ***************************************************/
    void ece353_MKII_Buzzer_Off(void){
        // Turn off the timer
        TIMER_A0->CTL &= ~TIMER_A_CTL_MC_MASK;
    }

    /***************************************************
     * Check Buzzer Run Status
     *
     * Parameters
     *      None
     * Returns
     *      True : TimerA0 is On
     *      False : TimerA0 is off
     ***************************************************/
    bool ece353_MKII_Buzzer_Run_Status(void){
        if( (TIMER_A0->CTL & TIMER_A_CTL_MC_MASK) == TIMER_A_CTL_MC__STOP){
            return false;
        }
        else{
            return true;
        }
    }

    //*****************************************************************************
    //*****************************************************************************
    // ICE 04 - PWMing MKII tri-color LED.
    //*****************************************************************************
    //*****************************************************************************

    /*****************************************************
    * Initialize the 3 GPIO pins that control the RGB * LED on the MKII.
    *
    * Parameters
    *      None
    * Returns
    *      None
    *****************************************************/
    void ece353_MKII_RGB_IO_Init(bool en_primary_function){
        // Complete the comments below to identify which pins
        // control which LEDs.

        // RED      : P2.6 : TA0.3
        // GREEN    : P2.4 : TA0.1
        // BLUE     : P5.6 : TA2.1

        // ADD CODE that configures the RED, GREEN, and
        // BLUE LEDs to be outputs
        P2->DIR |= BIT6;
        P2->DIR |= BIT4;
        P5->DIR |= BIT6;

        // ADD CODE that selects the Primary module function
        // for all 3 pins
        if(en_primary_function){
            // Red
            P2->SEL0 |= BIT6;
            P2->SEL1 &= ~BIT6;

            // Green
            P2->SEL0 |= BIT4;
            P2->SEL1 &= ~BIT4;

            // Blue
            P5->SEL0 |= BIT6;
            P5->SEL1 &= ~BIT6;
        }

        P2->OUT &= ~BIT6;
        P2->OUT &= ~BIT4;
        P5->OUT &= ~BIT6;

    }

    /*****************************************************
     *Sets the PWM levels for the MKII RGBLED
     *Parameters
     *  ticks_period    :   Period of PWM Pulse
     *  ticks_red_on    :   Number of Ticks RED is on
     *  ticks_green_on  :   Number of Ticks GREEN is on
     *  ticks_blue_on   :   Number of Ticks BLUE is on
     *
     * Returns
     *   None
     *****************************************************/
    void ece353_MKII_RGB_PWM(uint16_t ticks_period, uint16_t ticks_red_on,
                             uint16_t ticks_green_on, uint16_t ticks_blue_on){

        ece353_MKII_RGB_IO_Init(true);

        // Setting the three TimerA peripherals
        // Red->TA0.3, G->TA0.1, B->TA2.1
        // Stopping TimerA
        TIMER_A0->CTL = 0;
        TIMER_A2->CTL = 0;

        // Setting Clk to be master clk
        TIMER_A0->CTL |= TIMER_A_CTL_SSEL__SMCLK;
        TIMER_A2->CTL |= TIMER_A_CTL_SSEL__SMCLK;

        // Setting TIMERA to compare mode
        TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CAP;
        TIMER_A0->CCTL[3] &= ~TIMER_A_CCTLN_CAP;
        TIMER_A2->CCTL[1] &= ~TIMER_A_CCTLN_CAP;

        // Setting output to high
        TIMER_A0->CCTL[1] |= TIMER_A_CCTLN_OUTMOD_7;
        TIMER_A0->CCTL[3] |= TIMER_A_CCTLN_OUTMOD_7;
        TIMER_A2->CCTL[1] |= TIMER_A_CCTLN_OUTMOD_7;

        // Loading total ticks into timer
        TIMER_A0->CCR[0] = ticks_period-1;
        TIMER_A2->CCR[0] = ticks_period-1;

        // Loading compare ticks into timer
        TIMER_A0->CCR[1] = ticks_green_on;
        TIMER_A0->CCR[3] = ticks_red_on;
        TIMER_A2->CCR[1] = ticks_blue_on;

        // Start timerA
        TIMER_A0->CTL |= TIMER_A_CTL_MC__UP;
        TIMER_A2->CTL |= TIMER_A_CTL_MC__UP;
    }

    /*****************************************************
    * Configures Timer32_1 to generate a periodic interrupt
    *
    * Parameters
    *      ticks : Number of milliseconds per interrupt
    * Returns
    *      None
    *****************************************************/
    void ece353_T32_1_Interrupt_Ms(uint16_t ms){
        // Ticks = desired period / core clock period
        uint32_t ticks = (ms * SystemCoreClock / 1000) - 1;

        // Stop the timer
        TIMER32_1->CONTROL = 0;

        // Set the load register
        TIMER32_1->LOAD = ticks;

        // Enable the Timer32 interrupt in NVIC
        __enable_irq(); // Turns on interrupts
        NVIC_EnableIRQ(T32_INT1_IRQn); // Enable the Interrupt to be read
        NVIC_SetPriority(T32_INT1_IRQn, 0); // Set priority

        // Start Timer32 and enable interrupt
        TIMER32_1->CONTROL =    TIMER32_CONTROL_ENABLE |    // Turn timer on
                                TIMER32_CONTROL_MODE |      // Periodic Mode
                                TIMER32_CONTROL_SIZE |      // 32 bit timer
                                TIMER32_CONTROL_IE;         // Enable interrupts
    }

    void ece353_ADC14_PS2_X(void){
        // Configure P6.0 (A15) the X direction as an analog input pin.
        P6->SEL0 |= BIT0;
        P6->SEL1 |= BIT0;

        // Configure the ADC14 Control Registers
        // Sample time of 16 ADC cycles for the first 8 analog channels
        // Allow the ADC timer to control pulsed samples
        ADC14->CTL0 = ADC14_CTL0_SHP | ADC14_CTL0_SHT02;

        // Use sampling timer, 12-bit conversion results
        ADC14->CTL1 = ADC14_CTL1_RES_2;

        // Configure Memory Control register so that we associate the correct
        // analog channel with MEM[0]
        ADC14->MCTL[0] = ADC14_MCTLN_INCH_15;

        // Enable Interrupts after the conversion of MEM[0] completes
        ADC14->IER0 = ADC14_IER0_IE0;

        // Enable ADC Interrupt
        NVIC_EnableIRQ(ADC14_IRQn);

        // Turn ADC on
        ADC14->CTL0 |= ADC14_CTL0_ON;
    }

    /*****************************************************
     * Initialize hardware resources for the RGBLED on the
     * Launchpad
     *****************************************************/
    void ece353_RGB_LED_Init(void){
        // Set the 3 pins that control the RGB LED as outputs
        P2->DIR |= BIT2 | BIT1 | BIT0;

        P2->OUT = 0;
    }

    /******************************************************
     * Turn RGB LED ON/OFF
     *
     * Parameters
     *  red:    if true, turn RED LED on
     *          if false, turn RED LED off
     *  green:  if true, turn GREEN LED on
     *          if false, turn GREEN LED off
     *  blue:   if true, turn BLUE LED on
     *          if false, turn BLUE LED off
     *****************************************************/
    void ece353_RGB_LED(bool red, bool green, bool blue){
        if(red){
            P2->OUT |= BIT0;
        }
        else{
            P2->OUT &= ~BIT0;
        }

        if(green){
            P2->OUT |= BIT1;
        }
        else{
            P2->OUT &= ~BIT1;
        }

        if(blue){
            P2->OUT |= BIT2;
        }
        else{
            P2->OUT &= ~BIT2;
        }
    }

    /******************************************************************************
     *
     * * Configure the IO pins for BOTH the X and Y directions of the analog
     * * joystick.  The X direction should be configured to place the results in
     * * MEM[0].  The Y direction should be configured to place the results in MEM[1].
     *
     *  * After BOTH analog signals have finished being converted, a SINGLE interrupt
     *  * should be generated.
     *  *
     *  * Parameters
     *  *      None
     *  * Returns
     *  *      None
     *  ******************************************************************************/
    void ece353_ADC14_PS2_XY(void){
        // Configure the X direction as an analog input pin.
        P6->SEL0 |= BIT0;
        P6->SEL1 |= BIT0;

        // Configure the Y direction as an analog input pin.
        P4->SEL0 |= BIT4;
        P4->SEL1 |= BIT4;

        // Configure CTL0 to sample 16-times in pulsed sample mode.
        // NEW -- Indicate that this is a sequence of samples.
        ADC14->CTL0 = ADC14_CTL0_SHP; // ? 1b = SAMPCON signal is sourced from the sampling timer.
        ADC14->CTL0 |= ADC14_CTL0_SHT0_2; // ? 0010b = 16
        ADC14->CTL0 |= ADC14_CTL0_CONSEQ_1; // 01b = Sequence-of-channels

        // Configure ADC to return 12-bit values
        ADC14->CTL1 = ADC14_CTL1_RES_2; // 10b = 12 bit (14 clock cycle conversion time)

        // Associate the X direction analog signal with MEM[0]
        ADC14->MCTL[0] = ADC14_MCTLN_INCH_15;

        // Associate the Y direction analog signal with MEM[1]
        // NEW -- Make sure to indicate this is the end of a sequence.
        ADC14->MCTL[1] = ADC14_MCTLN_INCH_9;
        ADC14->MCTL[1] |= ADC14_MCTLN_EOS;

        // Enable interrupts in the ADC AFTER a value is written into MEM[1].
        //
        // NEW: This is not the same as what is demonstrated in the example
        // coding video.
        ADC14->IER0 = ADC14_IER0_IE1; // Interrupt enable. Enables or disables the interrupt request for the ADC14IFG1 bit.

        // Enable ADC Interrupt in the NVIC
        NVIC_EnableIRQ(ADC14_IRQn);

        // Turn ADC ON
        ADC14->CTL0 |= ADC14_CTL0_ON;

        }

    /***************************************************** *
     *  Turn RGB on the MKII LED ON/Off.
     *  *
     *  * Parameters
     *  *  red:    if true,  turn RED LED on
     *  *          if false, turn RED LED off
     *  *  green:  if true,  turn GREEN LED on
     *  *          if false, turn GREEN LED off
     *  *  blue:   if true,  turn BLUE LED on
     *  *          if false, turn BLUE LED off
     *  *****************************************************/
    void ece353_MKII_RGB_LED(bool red, bool green, bool blue){
        if(red){
            P2->OUT |= BIT6;
        }
        else{
            P2->OUT &= ~BIT6;
        }

        if(green){
            P2->OUT |= BIT4;
        }
        else{
            P2->OUT &= ~BIT4;
        }

        if(blue){
            P5->OUT |= BIT6;
        }
        else{
            P5->OUT &= ~BIT6;
        }
    }

    //*****************************************************************************
    //*****************************************************************************
    // ICE 07 - ADC14 Comparator
    //*****************************************************************************
    #define VOLT_0P85 1056      // 0.85 /(3.3/4096)
    #define VOLT_2P50 3103      // 2.50 /(3.3/4096)
    /******************************************************************************
     * * Configure the IO pins for the X direction of the analog
     * * joystick.  The X direction should be configured to place the results in
     * * MEM[0].
     * *
     * * The ADC14 should be configured to generate interrupts using the Window
     * * comparator.  The HI0 threshold should be set to 2.5V.  The LO0 threshold
     * * should be set to 0.85V.
     * *
     * * Parameters
     * *      None
     * * Returns
     * *      None
     * ******************************************************************************/
    void ece353_ADC14_PS2_XY_COMP(void){
        // Configure the X direction as an analog input pin.
        P6->SEL0 |= BIT0;
        P6->SEL1 |= BIT0;

        // Configure CTL0 to sample 16-times in pulsed sample mode.
        // Indicate that this is a sequence of samples.
        ADC14->CTL0 = ADC14_CTL0_SHP; // ? 1b = SAMPCON signal is sourced from the sampling timer.
        ADC14->CTL0 |= ADC14_CTL0_SHT0_2; // ? 0010b = 16
        ADC14->CTL0 |= ADC14_CTL0_CONSEQ_1; // 01b = Sequence-of-channels
        ADC14->CTL0 &= ~ADC14_CTL0_ENC; // Allows to write to HI0 and LO0 when zeroed

        // Configure CTL1 to return 12-bit values
        ADC14->CTL1 = ADC14_CTL1_RES_2; // 10b = 12 bit (14 clock cycle conversion time)
        ADC14->CTL1 &= ~ADC14_CTL1_DF; // Using unsigned representation for comparitor

        // Set up the HI0 Window
        ADC14->HI0 = VOLT_2P50;

        // Set up the LO0 Window
        ADC14->LO0 = VOLT_0P85;

        // Associate the X direction analog signal with MEM[0], indicate the end of sequence,
        // turn on the window comparator
        ADC14->MCTL[0] = ADC14_MCTLN_INCH_15;
        ADC14->MCTL[0] |= ADC14_MCTLN_WINC;
        ADC14->MCTL[0] |= ADC14_MCTLN_EOS;

        // Enable interrupts when either the HI or LO thresholds of the window
        // comparator has been exceeded.  Disable all other interrupts
        ADC14->IER0 = 0;
        ADC14->IER1 = ADC14_IER1_HIIE | ADC14_IER1_LOIE;

        // Enable ADC Interrupt
        NVIC_EnableIRQ(ADC14_IRQn);

        // Turn ADC ON
        ADC14->CTL0 |= ADC14_CTL0_ON;
    }
