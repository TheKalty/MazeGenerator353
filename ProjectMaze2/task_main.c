#include "main.h"

TaskHandle_t Task_Main_Handle;

QueueHandle_t Queue_Maze;
QueueHandle_t Queue_Player;

void Init_Queue(){
    Queue_Maze = xQueueCreate(1,sizeof(struct Maze));

    Queue_Generated_Maze = xQueueCreate(1,sizeof(struct Maze));

    Queue_Player = xQueueCreate(1,sizeof(struct Maze));
}

void Init_Main(){
    // LCD Init
    Crystalfontz128x128_Init();

    // Joystick
    Task_Joystick_Init();

    // Queues
    Init_Queue();
}



void Task_Main(void *pvParameters){
    BaseType_t xHigherPriorityTaskWoken;
    struct Maze maze;

    while(1){
        // Init cycle
        vTaskNotifyGiveFromISR(Task_Gen_Maze_Handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

        while(1){
            // Get new maze
            xQueueReceive(Queue_Maze, &maze, portMAX_DELAY);

            // Send new maze to bound check for player
            xQueueSendToBack(Queue_Player, &maze, portMAX_DELAY);
        }

    }

}
