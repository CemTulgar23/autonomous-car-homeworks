#include <iostream>
#include <optional>
#include <limits>
#include "matrix.h"

Matrix createNewMatrix(){
    int row, column;
    std::cout << "Enter Matrix Row: " << std::endl;
    std::cin >> row;
    std::cout << "Enter Matrix Column: " << std::endl;
    std::cin >> column; 
    Matrix matrix(row, column);
    return matrix;
}

int main() {
    try
    {   
        Matrix matrix1 = createNewMatrix();
        matrix1.giveMatrixParameters();
        matrix1.printMatrix();
        Matrix matrix2 = createNewMatrix();
        matrix2.giveMatrixParameters();
        matrix2.printMatrix();
        matrix1.trace();
        std::cout << "Determinant: " << matrix1.determinant().value() << std::endl;
        matrix1.transpose();
        Matrix::add(matrix1, matrix2);
        Matrix::subtract(matrix1, matrix2);
        matrix1.neg();
        matrix1.cofactor();
        matrix1.inverse();
        matrix1.magnitude();
        Matrix::multiply(matrix1, matrix2);
        Matrix::dot(matrix1, matrix2);
        

    }
    catch(const std::exception& e)
    {

    }

    

}

