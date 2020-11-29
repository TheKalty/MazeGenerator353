#ifndef TASK_GENMAZE_H_
#define TASK_GENMAZE_H_

#include "main.h"

#define MAX_ROWS 10 // Max 255?
#define MAX_COLS 10 // Max 255?

// CELL Struct
struct Cell {
    bool wall[4]; // R, U, L, D
    bool visited; // For help with generation
};

struct Maze {
    struct Cell cells[MAX_COLS][MAX_ROWS]; // 10 by 10 cells for maze
};

struct Stack {
    uint8_t col[MAX_COLS*MAX_ROWS]; // Worst Case Stack
    uint8_t row[MAX_COLS*MAX_ROWS];
    int top; // Holds index of top
};

extern TaskHandle_t Task_Gen_Maze_Handle;

extern QueueHandle_t Queue_Generated_Maze;

// Generates the Maze, only function able to modify maze
void task_generateMaze(void *pvParameters);

#endif
