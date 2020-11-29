#include "main.h"

TaskHandle_t Task_Maze_Handle;

// Prints the Maze
void Task_Maze(void *pvParameters){
    struct Maze maze;

    while(1){

        // Wait till notification
        xQueueReceive(Queue_Generated_Maze, &maze, portMAX_DELAY);

        // Clear LCD
        lcd_draw_rectangle(66,66,132,132,LCD_COLOR_BLACK);

        // Loop to print maze
        uint8_t i, j, x, y;
        for( i = 0; i < MAX_ROWS; i++){
            for( j = 0; j < MAX_COLS; j++){
                if( maze.cells[j][i].wall[0] ){ // Right Wall
                    x = 11*j + 10;
                    y = 11*i + 5;
                    lcd_draw_rectangle(x,y,1,11,LCD_COLOR_BLUE);
                }

                if( maze.cells[j][i].wall[1] ){ // UP Wall
                    x = 11*j + 5;
                    y = 11*i + 0;
                    lcd_draw_rectangle(x,y,11,1,LCD_COLOR_BLUE);
                }

                if( maze.cells[j][i].wall[2] ){ // Left Wall
                    x = 11*j + 0;
                    y = 11*i + 5;
                    lcd_draw_rectangle(x,y,1,11,LCD_COLOR_BLUE);
                }

                if( maze.cells[j][i].wall[3] ){ // Down Wall
                    x = 11*j + 5;
                    y = 11*i + 10;
                    lcd_draw_rectangle(x,y,11,1,LCD_COLOR_BLUE);
                }
            }
        }

        // Red Starting square
        lcd_draw_rectangle(5,5,9,9,LCD_COLOR_RED);

        // Green Ending Square
        lcd_draw_rectangle(104,104,9,9,LCD_COLOR_GREEN);

        // Send to main
        xQueueSendToBack(Queue_Maze, &maze, portMAX_DELAY);
    }

}
