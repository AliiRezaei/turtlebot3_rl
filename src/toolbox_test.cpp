#include "turtlebot3_rl/Toolbox.h"
#include <iostream>

int main() {
    std::size_t row = 4;
    std::size_t col = 2;
    float matrix[row][col] = {
        {10.0, 11.0},
        {20.0, 21.0},
        {30.0, 31.0},
        {40.0, 41.0}
    };

    // Create an array of pointers to each row of the matrix
    float *matrix_ptr[row];
    for (std::size_t i = 0; i < row; ++i) {
        matrix_ptr[i] = matrix[i];
    }

    float arr[col] = {10.0, 11.0};

    int result = ismember<float>(arr, matrix_ptr, row, col);

    if (result != -1) {
        std::cout << "Array is a member of the matrix at row index: " << result << std::endl;
    } else {
        std::cout << "Array is not a member of the matrix" << std::endl;
    }

    return 0;
}