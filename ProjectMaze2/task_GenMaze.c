#include "main.h"

TaskHandle_t Task_Gen_Maze_Handle;

QueueHandle_t Queue_Generated_Maze;

bool cellHasNeighboors(struct Maze* maze, uint8_t currCol, uint8_t currRow ) {
    // Check Right
    if (currCol != MAX_COLS-1 ){
        if (!maze->cells[currCol+1][currRow].visited) {
            return true;
        }
    }

    // Check Left
    if (currCol != 0 ){
        if (!maze->cells[currCol-1][currRow].visited) {
            return true;
        }
    }

    // Check Up
    if (currRow != 0 ){
        if (!maze->cells[currCol][currRow-1].visited) {
            return true;
        }
    }

    // Check Down
    if (currRow != MAX_ROWS-1 ){
        if (!maze->cells[currCol][currRow+1].visited) {
            return true;
        }
    }

    return false;
}

uint8_t randDir() {
    return rand() % 4;
}

void task_generateMaze(void *pvParameters) {
    struct Stack stack;
    struct Maze maze;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int i, j;

    while(1){

        // Wait to be notified
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // Zero Stack
            for ( i = 0; i < MAX_COLS*MAX_ROWS; i++) {
                stack.row[i] = 0;
                stack.col[i] = 0;
            }
            stack.top = 0;

            // Mark all Cells as not visited
            for ( i = 0; i < MAX_ROWS; i++) {
                for ( j = 0; j < MAX_COLS; j++) {
                    // Init Walls to true
                    maze.cells[j][i].wall[0] = true;
                    maze.cells[j][i].wall[1] = true;
                    maze.cells[j][i].wall[2] = true;
                    maze.cells[j][i].wall[3] = true;

                    // Visited to false
                    maze.cells[j][i].visited = false;
                }
            }

            // Choose the initial cell, mark it as visited and push it to the stack
            // Initial Cell is top left, aka cells[0][0]
            maze.cells[0][0].visited = true;
            stack.col[stack.top] = 0;
            stack.row[stack.top] = 0;
            stack.top = 0;

            // While the stack is not empty
            uint8_t currCellCol, currCellRow;
            while (stack.top != -1) {
                //Pop a cell from the stackand make it a current cell
                currCellCol = stack.col[stack.top];
                currCellRow = stack.row[stack.top];
                stack.top--; // Init is -1

                // If the current cell has any neighbours which have not been visited
                if (cellHasNeighboors(&maze, currCellCol, currCellRow)) {
                    // Push the current cell to the stack
                    stack.top++;
                    stack.col[stack.top] = currCellCol;
                    stack.row[stack.top] = currCellRow;

                    // Choose one of the unvisited neighbours
                    uint8_t dir = randDir();
                    bool validDir = false;
                    while (!validDir) {
                        switch (dir) {
                        case 0: // R
                            if (currCellCol != MAX_COLS - 1) {
                                if (!maze.cells[currCellCol+1][currCellRow].visited) {
                                    validDir = true;
                                    break;
                                }
                                dir = randDir();
                            }
                            else {
                                dir = randDir();
                            }
                            break;

                        case 1: // L
                            if (currCellCol != 0) {
                                if (!maze.cells[currCellCol-1][currCellRow].visited) {
                                    validDir = true;
                                    break;
                                }
                                dir = randDir();
                            }
                            else {
                                dir = randDir();
                            }
                            break;

                        case 2: // U
                            if (currCellRow != 0) {
                                if (!maze.cells[currCellCol][currCellRow - 1].visited) {
                                    validDir = true;
                                    break;
                                }
                                dir = randDir();
                            }
                            else {
                                dir = randDir();
                            }
                            break;

                        case 3: // D
                            if (currCellRow != MAX_ROWS - 1) {
                                if (!maze.cells[currCellCol][currCellRow + 1].visited) {
                                    validDir = true;
                                    break;
                                }
                                dir = randDir();
                            }
                            else {
                                dir = randDir();
                            }
                            break;

                        default:
                            dir = randDir();
                            break;
                        }
                    }

                        // Remove the wall between the current cell and the chosen cell
                        // &
                        // Mark the chosen cell as visited and push it to the stack
                        switch (dir) {
                        case 0: // R
                            maze.cells[currCellCol][currCellRow].wall[0] = false;
                            maze.cells[currCellCol + 1][currCellRow].wall[2] = false;

                            maze.cells[currCellCol + 1][currCellRow].visited = true;

                            stack.top++;
                            stack.col[stack.top] = currCellCol+1;
                            stack.row[stack.top] = currCellRow;

                            break;

                        case 1: // L
                            maze.cells[currCellCol][currCellRow].wall[2] = false;
                            maze.cells[currCellCol-1][currCellRow].wall[0] = false;

                            maze.cells[currCellCol-1][currCellRow].visited = true;

                            stack.top++;
                            stack.col[stack.top] = currCellCol-1;
                            stack.row[stack.top] = currCellRow;
                            break;

                        case 2: // U
                            maze.cells[currCellCol][currCellRow].wall[1] = false;
                            maze.cells[currCellCol][currCellRow-1].wall[3] = false;

                            maze.cells[currCellCol][currCellRow-1].visited = true;

                            stack.top++;
                            stack.col[stack.top] = currCellCol;
                            stack.row[stack.top] = currCellRow-1;
                            break;

                        case 3: // D
                            maze.cells[currCellCol][currCellRow].wall[3] = false;
                            maze.cells[currCellCol][currCellRow+1].wall[1] = false;

                            maze.cells[currCellCol][currCellRow+1].visited = true;

                            stack.top++;
                            stack.col[stack.top] = currCellCol;
                            stack.row[stack.top] = currCellRow+1;
                            break;

                        default:
                            break;
                        }

                }


            }

        // Notify to print screen and main
        xQueueSendToBack(Queue_Generated_Maze, &maze, portMAX_DELAY);
    }
}
