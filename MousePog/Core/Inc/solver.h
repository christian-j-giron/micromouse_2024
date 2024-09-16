#ifndef SOLVER_H
#define SOLVER_H

typedef enum Heading {EAST, NORTH, WEST, SOUTH} Heading;
typedef enum Action {LEFT, FORWARD, RIGHT, BACK, IDLE} Action;

Action floodFill(void);
Action floodFillReturn(void);

void initMaze(void);
void updateMaze(void);
void updateNeighbors(void);
void updateMazeReturn(void);
void resetPosition(void);
void resetPositionReturn(void);

#endif
