#include <vector>
#include <iostream>
#include <limits>
#include <cmath>
#include "matrix.h"

Matrix::Matrix()
{
   matrix.resize(0);
}

Matrix::Matrix(int row, int column)
{
   matrix.resize(row, std::vector<double>(column, 0));
   // resize() fonksyionu ile önceden oluşturulan vector'ü boyutlandırabiliyoruz.
}

Matrix::Matrix(std::vector<std::vector<double>> mat) : matrix(mat)
{
}

std::vector<std::vector<double>> Matrix::getMatrix()
{
   return matrix;
}

void Matrix::printMatrix()
{
   for (int i = 0; i < matrix.size(); i++)
   {
      std::cout << "{ ";
      for (int j = 0; j < matrix[i].size(); j++)
      {
         std::cout << matrix[i][j];
         if (j + 1 != matrix[i].size())
         {
            std::cout << ", ";
         }
      }
      std::cout << " }" << std::endl;
   }
}

void Matrix::giveMatrixParameters()
{
   std::string input;
   int number = 1;
   for (int i = 0; i < matrix.size(); i++)
   {
      for (int j = 0; j < matrix[i].size(); j++)
      {
         std::cout << "Please enter " << number << ". " << "parameter of the matrix: ";
         std::cin >> input;
         while (true)
         {
            try
            {
               double num = std::stod(input);
               matrix[i][j] = num;
               number++;
               break;
            }
            catch (const std::invalid_argument &e)
            {
               std::cout << "Invalid input. Please enter a valid number." << std::endl;
               std::cin.clear();
               std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
               std::cout << "Please enter " << number << ". " << "parameter of the matrix: ";
               std::cin >> input;
            }
            catch (const std::out_of_range &e)
            {
               std::cout << "The entered number is out of range. Try again." << std::endl;
               std::cin.clear();
               std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
               std::cout << "Please enter " << number << ". " << "parameter of the matrix: ";
               std::cin >> input;
            }
         }
      }
   }
}

std::optional<double> Matrix::trace()
{
   if (matrix.size() != matrix[0].size())
   {
      std::cout << "Only square matrix's trace can be calculated." << std::endl;
      return std::nullopt; // ya nullopt ya da int döndürmek zorunda çünkü
   }
   else
   {
      double trace = 0;
      for (int i = 0; i < matrix.size(); i++)
      {
         trace = trace + matrix[i][i];
      }
      std::cout << "Trace: " << trace << std::endl;
      return trace;
   }
}

std::optional<double> Matrix::determinant()
{
   if (matrix.size() != matrix[0].size())
   {
      std::cout << "Only square matrix's determinat can be calculated." << std::endl;
      return std::nullopt;
   }
   else
   {
      if (matrix.size() > 3)
      {
         std::cout << "You can calculate only two and three square matrix's determinant in this program." << std::endl;
         return std::nullopt;
      }
      if (matrix.size() == 1)
      {
         double determinant = matrix[0][0];
         // std::cout << "Determinant: " << determinant << std::endl;
         return determinant;
      }
      if (matrix.size() == 2)
      {
         double determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
         // std::cout << "Determinant: " << determinant << std::endl;
         return determinant;
      }
      else
      {
         double determinant = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
                              matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
                              matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
         // std::cout << "Determinant: " << determinant << std::endl;
         return determinant;
      }
   }
}

Matrix Matrix::transpose()
{
   Matrix newMatrix(matrix[0].size(), matrix.size());
   for (int i = 0; i < matrix.size(); i++)
   {
      for (int j = 0; j < matrix[0].size(); j++)
      {
         newMatrix.matrix[j][i] = matrix[i][j];
      }
   }
   std::cout << "Transpose Matrix:" << std::endl;
   newMatrix.printMatrix();
   return newMatrix;
}

std::optional<Matrix> Matrix::add(Matrix matrix1, Matrix matrix2)
{
   if (matrix1.matrix.size() != matrix2.matrix.size() || matrix1.matrix[0].size() != matrix2.matrix[0].size())
   {
      std::cout << "For add two matrix, matrix's sizes must be equal." << std::endl;
      return std::nullopt;
   }
   else
   {
      Matrix newMatrix(matrix1.matrix.size(), matrix1.matrix[0].size());
      for (int i = 0; i < matrix1.matrix.size(); i++)
      {
         for (int j = 0; j < matrix1.matrix[0].size(); j++)
         {
            newMatrix.matrix[i][j] = matrix1.matrix[i][j] + matrix2.matrix[i][j];
         }
      }
      std::cout << "Addition Matrix: " << std::endl;
      newMatrix.printMatrix();
      return newMatrix;
   }
}

std::optional<Matrix> Matrix::subtract(Matrix matrix1, Matrix matrix2)
{
   if (matrix1.matrix.size() != matrix2.matrix.size() || matrix1.matrix[0].size() != matrix2.matrix[0].size())
   {
      std::cout << "For substract two matrix, matrix's sizes must be equal." << std::endl;
      return std::nullopt;
   }
   else
   {
      Matrix newMatrix(matrix1.matrix.size(), matrix1.matrix[0].size());
      for (int i = 0; i < matrix1.matrix.size(); i++)
      {
         for (int j = 0; j < matrix1.matrix[0].size(); j++)
         {
            newMatrix.matrix[i][j] = matrix1.matrix[i][j] - matrix2.matrix[i][j];
         }
      }
      std::cout << "Substract Matrix: " << std::endl;
      newMatrix.printMatrix();
      return newMatrix;
   }
}

Matrix Matrix::neg()
{
   Matrix negMatrix(matrix.size(), matrix[0].size());
   for (int i = 0; i < matrix.size(); i++)
   {
      for (int j = 0; j < matrix[0].size(); j++)
      {
         negMatrix.matrix[i][j] = -matrix[i][j];
      }
   }
   std::cout << "Neg Matrix" << std::endl;
   negMatrix.printMatrix();
   return negMatrix;
}

std::optional<Matrix> Matrix::inverse()
{
   if (matrix.size() != matrix[0].size())
   {
      std::cout << "Only square matrix can be inversed." << std::endl;
      return std::nullopt;
   }
   else
   {
      int determinant = this->determinant().value();
      if (determinant == 0)
      {
         std::cout << "Matrix can't be inversed because determinant is zero." << std::endl;
         return std::nullopt;
      }
      else
      {
         Matrix cofactorMatrix = this->cofactor();
         Matrix adjointMatrix = cofactorMatrix.transpose();
         Matrix inverseMatrix = adjointMatrix.multiplyWithdouble(1.0 / determinant);
         std::cout << "Inverse Matrix: " << std::endl;
         inverseMatrix.printMatrix();
         return inverseMatrix;
      }
   }
}

Matrix Matrix::cofactor()
{
   // Her eleman için bir alt matris oluştur ve işlemleri uygula
   Matrix cofactorMatrix(matrix.size(), matrix[0].size());
   for (int excludedRow = 0; excludedRow < matrix.size(); excludedRow++)
   {
      for (int excludedColumn = 0; excludedColumn < matrix[0].size(); excludedColumn++)
      {
         Matrix minorMatrix(matrix.size() - 1, matrix[0].size() - 1);
         int minorRow = 0;
         for (int row = 0; row < matrix.size(); row++)
         {
            if (row == excludedRow)
               continue; // Hariç tutulan satır
            int minorColumn = 0;
            for (int column = 0; column < matrix[0].size(); column++)
            {
               if (column == excludedColumn)
                  continue; // Hariç tutulan sütun
               minorMatrix.matrix[minorRow][minorColumn] = this->matrix[row][column];
               minorColumn++;
            }
            minorRow++;
         }

         //std::cout << "Minor Matris: " << std::endl;
         //minorMatrix.printMatrix();
         // Oluşturulan alt matrisi yazdır
         double determinant = minorMatrix.determinant().value();
         //std::cout << "(" << excludedRow+1 << ", " << excludedColumn+1 << std::endl;
         //std::cout << "Determinant: " << determinant << std::endl;
         if ((excludedColumn + excludedRow) % 2 == 0)
         {  
            cofactorMatrix.matrix[excludedRow][excludedColumn] = determinant;
         }
         else
         {  
            cofactorMatrix.matrix[excludedRow][excludedColumn] = -determinant;
         }
      }
   }
   std::cout << "Cofactor Matrix: " << std::endl;
   cofactorMatrix.printMatrix();
   return cofactorMatrix.matrix;
}

double Matrix::magnitude()
{
   double sum = 0;
   for (int i = 0; i < matrix.size(); i++)
   {
      for (int j = 0; j < matrix[0].size(); j++)
      {
         sum = sum + pow(matrix[i][j], 2);
      }
   }
   double magnitude = sqrt(sum);
   std::cout << "Magnitude: " << magnitude << std::endl;
   return magnitude;
}

std::optional<Matrix> Matrix::multiply(Matrix matrix1, Matrix matrix2)
{ // Eleman Bazlı Çarpım
   if (matrix1.matrix.size() == matrix2.matrix.size() && matrix1.matrix[0].size() == matrix2.matrix[0].size())
   {
      Matrix multiplyMatrix(matrix1.matrix.size(), matrix1.matrix[0].size());
      for (int i = 0; i < matrix1.matrix.size(); i++)
      {
         for (int j = 0; j < matrix1.matrix[0].size(); j++)
         {
            multiplyMatrix.matrix[i][j] = matrix1.matrix[i][j] * matrix2.matrix[i][j];
         }
      }
      std::cout << "Multiple Matrix" << std::endl;
      multiplyMatrix.printMatrix();
      return multiplyMatrix;
   }
   else
   {
      std::cout << "For multiplication two matrix's sizes must be equal." << std::endl;
      return std::nullopt;
   }
}

std::optional<Matrix> Matrix::dot(Matrix matrix1, Matrix matrix2)
{
   if (matrix1.matrix.size() == matrix2.matrix[0].size())
   {
      Matrix dotMatrix;
      for (int i = 0; i < matrix1.matrix.size(); i++)
      {
         for (int j = 0; j < matrix2.matrix[0].size(); j++)
         {
            double sum = 0;
            for (int index = 0; index < matrix1.matrix[0].size(); index++)
            {
               sum = sum + matrix1.matrix[i][index] * matrix2.matrix[index][j];
               if (dotMatrix.matrix.size() < i + 1)
               {
                  dotMatrix.matrix.resize(i + 1, {});
               }
               if (dotMatrix.matrix[i].size() < j + 1)
               {
                  dotMatrix.matrix[i].resize(j + 1);
               }
               dotMatrix.matrix[i][j] = sum;
            }
         }
      }
      std::cout << "Dot Matrix:" << std::endl;
      dotMatrix.printMatrix();
      return dotMatrix;
   }
   else
   {
      std::cout << "Matrix's dimensions aren't suitable for dot." << std::endl;
      return std::nullopt;
   }
}

Matrix Matrix::multiplyWithdouble(double value)
{
   Matrix resultMatrix(matrix.size(), matrix[0].size());
   for (int i = 0; i < matrix.size(); i++)
   {
      for (int j = 0; j < matrix[0].size(); j++)
      {
         resultMatrix.matrix[i][j] = matrix[i][j] * value;
      }
   }
   std::cout << "Multiply double Matrix:" << std::endl;
   resultMatrix.printMatrix();
   return resultMatrix;
}
