#ifndef TASK_MAIN_H_
#define TASK_MAIN_H_

#include "main.h"

extern TaskHandle_t Task_Main_Handle;
extern QueueHandle_t Queue_Maze;
extern QueueHandle_t Queue_Player;

void Task_Main(void *pvParameters);

void Init_Main();

#endif
