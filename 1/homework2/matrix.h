#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
#include <optional>

class Matrix {
    private:
    std::vector<std::vector<double>> matrix;
    
    public:
    Matrix();
    Matrix(int row, int column);
    Matrix(std::vector<std::vector<double>> mat);
    std::vector<std::vector<double>> getMatrix();
    void printMatrix();
    void giveMatrixParameters();
    std::optional<double> trace(); // Matrisin köşegenlerindeki sayıları topluyor
    std::optional<double> determinant();
    Matrix transpose(); // satırla sütunların yerlerini değiştiriyor
    static std::optional<Matrix> add(Matrix matrix1, Matrix matrix2);
    static std::optional<Matrix> subtract(Matrix matrix1, Matrix matrix2);
    Matrix neg();
    std::optional<Matrix> inverse(); // Matrisin tersini alıyor
    Matrix cofactor();
    double magnitude();
    static std::optional<Matrix> multiply(Matrix matrix1, Matrix matrix2);
    static std::optional<Matrix> dot(Matrix matrix1, Matrix matrix2);
    Matrix multiplyWithdouble(double value);
};

#endif