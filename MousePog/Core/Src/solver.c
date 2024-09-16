#include "solver.h"
#include "irs.h"

#define MAX_INDEX 500 
#define NUM_ROWS 16 
#define NUM_COLUMNS 16

int maze[NUM_ROWS][NUM_COLUMNS][4];
int m_dist[NUM_ROWS][NUM_COLUMNS]; 
int updated[NUM_ROWS][NUM_COLUMNS]; 

int pose = 1; 

int row = 15; 
int col = 0;


// Floodfill maze-solving algorithm implementation
Action floodFill(void) {

    if (readFrontLeftIR() > 400 && readFrontRightIR() > 400) {
        maze[row][col][pose] = 1; 
    }
    if (readLeftIR() > 400) {
        maze[row][col][(pose + 1 + 4) % 4] = 1; 
    }
    if (readRightIR() > 400) {
        maze[row][col][(pose + 3 + 4) % 4] = 1; 
    } 

    //BEGIN UPDATE NEIGHBORS
    updateNeighbors(); 
    //END UPDATE NEIGHBORS

    //BEGIN UPDATE MAZE 
    updateMaze();  
    //END UPDATE MAZE

    int min = m_dist[row][col]; 
    if (min == 0) { return IDLE; }

    int tRow = row; 
    int tCol = col; 
    int tDir = pose; 
    if (row != 0 && m_dist[row - 1][col] <= min && maze[row][col][NORTH] == 0) {
        tRow = row - 1; 
        tCol = col; 
        min = m_dist[tRow][tCol]; 
        tDir = 1;
    }
    if (row != 15 && m_dist[row + 1][col] <= min && maze[row][col][SOUTH] == 0) {
        tRow = row + 1; 
        tCol = col; 
        min = m_dist[tRow][tCol];
        tDir = 3; 
    }
    if (col != 0 && m_dist[row][col - 1] <= min && maze[row][col][WEST] == 0) {
        tRow = row; 
        tCol = col - 1; 
        min = m_dist[tRow][tCol]; 
        tDir = 2; 
    }
    if (col != 15 && m_dist[row][col + 1] <= min && maze[row][col][EAST] == 0) {
        tRow = row; 
        tCol = col + 1; 
        min = m_dist[tRow][tCol]; 
        tDir = 0; 
    } 

    row = tRow; 
    col = tCol; 

    int poseCorrection = (tDir - pose + 4) % 4; 
    pose = tDir; 
    switch (poseCorrection) {
        case 0:
            return FORWARD;
            break;
        case 1: 
            return LEFT; 
            break; 
        case 2: 
            return BACK; 
            break; 
        case 3: 
            return RIGHT; 
            break; 
    }
    return IDLE;
} 


Action floodFillReturn(void) {

	if (readFrontLeftIR() > 400 && readFrontRightIR() > 400 || readFrontLeftIR() > 650) {
		maze[row][col][pose] = 1;
	}
	if (readLeftIR() > 400) {
		maze[row][col][(pose + 1 + 4) % 4] = 1;
	}
	if (readRightIR() > 400) {
		maze[row][col][(pose + 3 + 4) % 4] = 1;
	}

    //BEGIN UPDATE NEIGHBORS
    updateNeighbors();
    //END UPDATE NEIGHBORS

    //BEGIN UPDATE MAZE
    updateMazeReturn();
    //END UPDATE MAZE

    int min = m_dist[row][col];
    if (min == 0) { return IDLE; }

    int tRow = row;
    int tCol = col;
    int tDir = pose;
    if (row != 0 && m_dist[row - 1][col] <= min && maze[row][col][NORTH] == 0) {
        tRow = row - 1;
        tCol = col;
        min = m_dist[tRow][tCol];
        tDir = 1;
    }
    if (row != 15 && m_dist[row + 1][col] <= min && maze[row][col][SOUTH] == 0) {
        tRow = row + 1;
        tCol = col;
        min = m_dist[tRow][tCol];
        tDir = 3;
    }
    if (col != 0 && m_dist[row][col - 1] <= min && maze[row][col][WEST] == 0) {
        tRow = row;
        tCol = col - 1;
        min = m_dist[tRow][tCol];
        tDir = 2;
    }
    if (col != 15 && m_dist[row][col + 1] <= min && maze[row][col][EAST] == 0) {
        tRow = row;
        tCol = col + 1;
        min = m_dist[tRow][tCol];
        tDir = 0;
    }

    row = tRow;
    col = tCol;

    int poseCorrection = (tDir - pose + 4) % 4;
    pose = tDir;
    switch (poseCorrection) {
        case 0:
            return FORWARD;
            break;
        case 1:
            return LEFT;
            break;
        case 2:
            return BACK;
            break;
        case 3:
            return RIGHT;
            break;
    }
    return IDLE;
}


void updateNeighbors(void) {
    if (row != 0) {
        maze[row - 1][col][SOUTH] = maze[row][col][NORTH];
    } 
    if (row != 15) {
        maze[row + 1][col][NORTH] = maze[row][col][SOUTH]; 
    } 
    if (col != 0) {
        maze[row][col - 1][EAST] = maze[row][col][WEST]; 
    } 
    if (col != 15) {
        maze[row][col + 1][WEST] = maze[row][col][EAST]; 
    }
}


void initMaze(void) {

    for (int r = 0; r < 16; r++) {
        for (int c = 0; c < 16; c++) { 
            
            maze[r][c][NORTH] = (r == 0)? 1 : 0;          
            maze[r][c][SOUTH] = (r == 15)? 1 : 0;           
            maze[r][c][WEST] = (c == 0)? 1 : 0;           
            maze[r][c][EAST] = (c == 15)? 1 : 0; 
            m_dist[r][c] = 0; 
             
        }
    } 

    for (int r = 0; r < 16; r++) {
        for (int c = 0; c < 16; c++) {
            updated[r][c] = 0; 
        }
    }

    updateMaze(); 

} 


void updateMaze(void) {

    int queue[MAX_INDEX][2]; 
    
    queue[0][0] = queue[0][1] = queue[1][0] = queue[2][1] = 7;
    queue[1][1] = queue[2][0] = queue[3][0] = queue[3][1] = 8; 
    updated[7][7] = updated[7][8] = updated[8][7] = updated[8][8] = 1; 

    int front = 0; 
    int back = 4; 

    int rowU; 
    int colU; 

    int dist = 0; 

    int tilesUpdated = 0;

    while (front != back) { 
        int increment_dist = back; 

        while (front != increment_dist) {

            rowU = queue[front][0];
            colU = queue[front][1]; 

            m_dist[rowU][colU] = dist; 
            
            if (maze[rowU][colU][NORTH] == 0 && updated[rowU - 1][colU] == 0) {                
                queue[back][0] = rowU - 1;  
                queue[back][1] = colU; 
                updated[rowU - 1][colU] = 1; 
                back = (back + 1) % MAX_INDEX;
            }
            if (maze[rowU][colU][SOUTH] == 0 && updated[rowU + 1][colU] == 0) {
                queue[back][0] = rowU + 1;  
                queue[back][1] = colU; 
                updated[rowU + 1][colU] = 1; 
                back = (back + 1) % MAX_INDEX; 
            }
            if (maze[rowU][colU][WEST] == 0 && updated[rowU][colU - 1] == 0) {
                queue[back][0] = rowU;  
                queue[back][1] = colU - 1; 
                updated[rowU][colU - 1] = 1; 
                back = (back + 1) % MAX_INDEX; 
            }
            if (maze[rowU][colU][EAST] == 0 && updated[rowU][colU + 1] == 0) {
                queue[back][0] = rowU;  
                queue[back][1] = colU + 1; 
                updated[rowU][colU + 1] = 1; 
                back = (back + 1) % MAX_INDEX; 
            }
            
            front = (front + 1) % MAX_INDEX;  
            tilesUpdated++; 
        } 

        dist++; 
    } 

    for (int r = 0; r < 16; r++) {
        for (int c = 0; c < 16; c++) {
            updated[r][c] = 0; 
        }
    }

} 

void updateMazeReturn(void) {

    int queue[MAX_INDEX][2];

    queue[0][0] = 15;
    queue[0][1] = 0;
    updated[15][0] = 1;

    int front = 0;
    int back = 1;

    int rowU;
    int colU;

    int dist = 0;

    int tilesUpdated = 0;

    while (front != back) {
        int increment_dist = back;

        while (front != increment_dist) {

            rowU = queue[front][0];
            colU = queue[front][1];

            m_dist[rowU][colU] = dist;


            if (maze[rowU][colU][NORTH] == 0 && updated[rowU - 1][colU] == 0) {
                queue[back][0] = rowU - 1;
                queue[back][1] = colU;
                updated[rowU - 1][colU] = 1;
                back = (back + 1) % MAX_INDEX;
            }
            if (maze[rowU][colU][SOUTH] == 0 && updated[rowU + 1][colU] == 0) {
                queue[back][0] = rowU + 1;
                queue[back][1] = colU;
                updated[rowU + 1][colU] = 1;
                back = (back + 1) % MAX_INDEX;
            }
            if (maze[rowU][colU][WEST] == 0 && updated[rowU][colU - 1] == 0) {
                queue[back][0] = rowU;
                queue[back][1] = colU - 1;
                updated[rowU][colU - 1] = 1;
                back = (back + 1) % MAX_INDEX;
            }
            if (maze[rowU][colU][EAST] == 0 && updated[rowU][colU + 1] == 0) {
                queue[back][0] = rowU;
                queue[back][1] = colU + 1;
                updated[rowU][colU + 1] = 1;
                back = (back + 1) % MAX_INDEX;
            }

            front = (front + 1) % MAX_INDEX;
            tilesUpdated++;
        }

        dist++;
    }

    for (int r = 0; r < 16; r++) {
        for (int c = 0; c < 16; c++) {
            updated[r][c] = 0;
        }
    }
}

void resetPosition(void) {
	row = 15;
	col = 0;
	pose = 0;
}

void resetPositionReturn(void) {
	row = 7;
	col = 7;
	pose = 0;
}

