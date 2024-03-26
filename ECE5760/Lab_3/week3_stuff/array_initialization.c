#include <stdio.h>
#include <stdlib.h> 
#include <math.h>
#define ROWS 85
#define COLS 85
#define H    0.125


// define the height calculation
float calculateHeight(int row, int col, float h, float step) {
    float result;
    float temp;
    temp = h - fmax(abs(row - (ROWS-1) / 2), abs(col - (COLS-1) / 2)) * step;
    if(temp < 0){
        temp = 0;
    }
    result = temp;
    return result;
}

int main() {
    float arr[ROWS][COLS];
    float step = H /(((ROWS-1)/2)) ;
    // initialization
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            arr[i][j] = calculateHeight(i, j, H, step); 
        }
    }

    // print
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            printf("%f ", arr[i][j]);
        }
        printf("\n");
    }

    return 0;
}
