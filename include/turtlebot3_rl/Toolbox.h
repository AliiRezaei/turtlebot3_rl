#ifndef _TOOLBOX_
#define _TOOLBOX_

#include <string>
#include <math.h>
#include <iostream>
#include <random>
#include <vector>

template <typename InputType, std::size_t N>
std::size_t array_length(InputType (&arr)[N]) {
    return N;
}

// begin randGen class :
class randomGen {
public:
    float randUniform(float low, float up);
    int   randInteger(int low, int up);
};

float randomGen::randUniform(float low, float up){
  std::random_device rd;
  std::mt19937 eng(rd());
  std::uniform_real_distribution<>distr_real(low, up);
  float rand = distr_real(eng);
  return rand;
}

int randomGen::randInteger(int low, int up){
  std::random_device rd;
  std::mt19937 eng(rd());
  std::uniform_int_distribution<>distr(low, up);
  int rand = distr(eng);
  return rand;
}
// end randGen class

// begin maxFind class :
class maxFind {
public:
    double maxVal;
    int    maxIdx;
    template <typename InputType> void maxFunc(InputType *arr, std::size_t len);
};

template <typename InputType> void maxFind::maxFunc(InputType *arr, std::size_t len) {
    double max_value = -100000.0;
    int    max_index = 0;  
    for(std::size_t i = 0; i < len; i++){
      if(*(arr + i) > max_value){
        max_value = *(arr + i);
        max_index = i;
      }
    }
    this->maxVal = max_value;
    this->maxIdx = max_index;
}
// end maxFind class

// begin minFind class :
class minFind {
public:
    double minVal;
    int    minIdx;
    template <typename InputType> void minFunc(InputType *arr, std::size_t len);
};

template <typename InputType> void minFind::minFunc(InputType *arr, std::size_t len) {
    double min_value = 100000.0;
    int    min_index = 0;  
    for(std::size_t i = 0; i < len; i++){
      if(*(arr + i) < min_value){
        min_value = *(arr + i);
        min_index = i;
      }
    }
    this->minVal = min_value;
    this->minIdx = min_index;
}

template<typename InputType> float *linspace(InputType a, InputType b, int n) {

    float step_size = (b - a) / (n - 1.0);
    float* sequence = new float[n];

    for(int i = 0; i < n; i++) {
        sequence[i] = a + i * step_size;
    }
    return sequence;
}
// end minFind class

// begin array sum calculator :
double sum_array(double arr[]) {

  double sum = 0.0;
  for(int i=0; i<5; i++) {
    sum += arr[i];
  }
  return sum;
}
// end array sum calculator

// begin two array eucliden distance calculator :
float eucliden_distance(float P1[], float P2[]) {

  float norm = 0.0;
  for(int i=0; i<2; i++) {
    norm += (P1[i] - P2[i]) * (P1[i] - P2[i]);
  }
  return sqrt(norm);
}
// end two array eucliden distance calculator

// begin ismember :
template <typename InputType> int ismember(InputType *arr, InputType *matrix[], std::size_t row, std::size_t col) {
    
    for (std::size_t i = 0; i < row; ++i) {
        bool row_match = true;
        for (std::size_t j = 0; j < col; ++j) {
            if (matrix[i][j] != arr[j]) {
                row_match = false;
                break;
            }
        }
        if (row_match) {
            return i; // return the index of the matching row
        }
    }
    return -1; // return -1 if no match is found
}
// end ismember

// begin repmat :
template<typename InputType>
std::vector<std::vector<InputType>> repmat(InputType *matrix[], std::size_t row, std::size_t col, int n, int m) {
    std::size_t result_row = row * n;
    std::size_t result_col = col * m;
    
    // initialize the result matrix
    std::vector<std::vector<InputType>> result(result_row, std::vector<InputType>(result_col));
    
    for (std::size_t i = 0; i < result_row; ++i) {
        for (std::size_t j = 0; j < result_col; ++j) {
            // compute the source element in the original matrix
            std::size_t orig_i = i % row;
            std::size_t orig_j = j % col;
            result[i][j] = matrix[orig_i][orig_j];
        }
    }
    return result;
}
// end repmat

#endif