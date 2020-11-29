/*
 * task_led.c
 *
 *  Created on: Oct 19, 2020
 *      Author: Joe Krachey
 */

// Nathan Kaltenbrun

#include <main.h>

 TaskHandle_t Task_Joystick_Handle;
 TaskHandle_t Task_Joystick_Timer_Handle;

 uint8_t posX = 0, posY = 0;

 uint32_t JOYSTICK_X_DIR = 0;
 uint32_t JOYSTICK_Y_DIR = 0;

/******************************************************************************
* This function will run the same configuration code that you would have
* written for HW02.
******************************************************************************/
 void Task_Joystick_Init(void)
 {
     // Configure the X direction as an analog input pin.
     P6->SEL0 |= BIT0;
     P6->SEL1 |= BIT0;

     // Configure the Y direction as an analog input pin.
     P4->SEL0 |= BIT4;
     P4->SEL1 |= BIT4;

     // Configure CTL0 to sample 16-times in pulsed sample mode.
     // NEW -- Indicate that this is a sequence of samples.
     ADC14->CTL0 = ADC14_CTL0_SHT0_2 | ADC14_CTL0_SHP | ADC14_CTL0_CONSEQ_1;

     // Configure CTL1 to return 12-bit values
     ADC14->CTL1 = ADC14_CTL1_RES_2;         // Use sampling timer, 12-bit conversion results

     // Associate the X direction analog signal with MEM[0]
     ADC14->MCTL[0] |= ADC14_MCTLN_INCH_15;

     // Associate the Y direction analog signal with MEM[1]
     // NEW -- Make sure to indicate this is the end of a sequence.
     ADC14->MCTL[1] |= ADC14_MCTLN_INCH_9 | ADC14_MCTLN_EOS;

     // Enable interrupts AFTER a value is written into MEM[1].
     ADC14->IER0 |= ADC14_IER0_IE1 ;

     // Enable ADC Interrupt
     NVIC_EnableIRQ(ADC14_IRQn);

     // Interrupt priorities must NOT be set to 0.  It is preferred if they are set to a value that is > than 1 too.
     NVIC_SetPriority(ADC14_IRQn,2);

     // Turn ADC ON
     ADC14->CTL0 |= ADC14_CTL0_ON;

 }

 /******************************************************************************
 * Used to start an ADC14 Conversion
 ******************************************************************************/
 void Task_Joystick_Timer(void *pvParameters)
 {
     while(1){
         /*
         * Start the ADC conversion
         */
         ADC14->CTL0 |= ADC14_CTL0_SC | ADC14_CTL0_ENC;

         /*
         * Delay 5mS
         */
         vTaskDelay(pdMS_TO_TICKS(50));
     }
 }


/******************************************************************************
* Bottom Half Task.  Examines the ADC data from the joy stick on the MKII
******************************************************************************/
void Task_Joystick_Bottom_Half(void *pvParameters){
    enum direction prevDir = CENTER, dir = CENTER;
    BaseType_t xHigherPriorityTaskWoken;
    struct Maze maze;

    while(1)
    {
        /* ADD CODE
         * Wait until we get a task notification from the ADC14 ISR
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Find out direction
        if(JOYSTICK_X_DIR < VOLT_0P85){ //
            dir = LEFT;
        }
        else if(JOYSTICK_X_DIR > VOLT_2P50){
            dir = RIGHT;
        }
        else if(JOYSTICK_Y_DIR < VOLT_0P85){
            dir = DOWN;
        }
        else if(JOYSTICK_Y_DIR > VOLT_2P50){
            dir = UP;
        }
        else{
            dir = CENTER;
        }

        // Wait for queue very short though
        xQueueReceive(Queue_Player, &maze, 10);

        // Make Player move based on maze, if a possible move
        // Remove prev Player pos if moved
        if(1){
            switch(dir){
                    case RIGHT:
                        if( !maze.cells[posX][posY].wall[0] ){
                            if(posX == 0 && posY == 0){
                                lcd_draw_rectangle(posX*11 + 5,posY*11+5,6,6,LCD_COLOR_RED);
                            }
                            else if(posX == 9 && posY == 9){
                                lcd_draw_rectangle(posX*11 + 5,posY*11+5,6,6,LCD_COLOR_GREEN);
                            }
                            else{
                                lcd_draw_rectangle(posX*11 + 5,posY*11+5,6,6,LCD_COLOR_BLACK);
                            }
                            posX++;
                        }
                        break;

                    case UP:
                        if( !maze.cells[posX][posY].wall[1] ){
                            if(posX == 0 && posY == 0){
                                lcd_draw_rectangle(posX*11 + 5,posY*11+5,6,6,LCD_COLOR_RED);
                            }
                            else if(posX == 9 && posY == 9){
                                lcd_draw_rectangle(posX*11 + 5,posY*11+5,6,6,LCD_COLOR_GREEN);
                            }
                            else{
                                lcd_draw_rectangle(posX*11 + 5,posY*11+5,6,6,LCD_COLOR_BLACK);
                            }
                            posY--;
                        }
                        break;

                    case LEFT:
                        if( !maze.cells[posX][posY].wall[2] ){
                            if(posX == 0 && posY == 0){
                                lcd_draw_rectangle(posX*11 + 5,posY*11+5,6,6,LCD_COLOR_RED);
                            }
                            else if(posX == 9 && posY == 9){
                                lcd_draw_rectangle(posX*11 + 5,posY*11+5,6,6,LCD_COLOR_GREEN);
                            }
                            else{
                                lcd_draw_rectangle(posX*11 + 5,posY*11+5,6,6,LCD_COLOR_BLACK);
                            }
                            posX--;
                        }
                        break;

                    case DOWN:
                        if( !maze.cells[posX][posY].wall[3] ){
                            if(posX == 0 && posY == 0){
                                lcd_draw_rectangle(posX*11 + 5,posY*11+5,6,6,LCD_COLOR_RED);
                            }
                            else if(posX == 9 && posY == 9){
                                lcd_draw_rectangle(posX*11 + 5,posY*11+5,6,6,LCD_COLOR_GREEN);
                            }
                            else{
                                lcd_draw_rectangle(posX*11 + 5,posY*11+5,6,6,LCD_COLOR_BLACK);
                            }
                            posY++;
                        }
                        break;

                    case CENTER:
                        break;
                    }

            // prevDir = dir;
        }

        // Draw Current player
        lcd_draw_rectangle(posX*11 + 5,posY*11+5,6,6,LCD_COLOR_WHITE);

        // Check if completed, If so generate new map
        if(posX == 9 && posY == 9){
            posX = 0;
            posY = 0;
            vTaskNotifyGiveFromISR(Task_Gen_Maze_Handle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
        }
    }
}


/******************************************************************************
* Top Half of ADC14 Handler.
******************************************************************************/
void ADC14_IRQHandler(void){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    JOYSTICK_X_DIR = ADC14->MEM[0]; // Read the value and clear the interrupt
    JOYSTICK_Y_DIR = ADC14->MEM[1]; // Read the value and clear the interrupt


    /* ADD CODE
     * Send a task notification to Task_Joystick_Bottom_Half
     */
    vTaskNotifyGiveFromISR(
            Task_Joystick_Handle,
            &xHigherPriorityTaskWoken
    );

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}



