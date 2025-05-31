#include "mat.h"

using namespace std;

mat::mat() {}
mat::mat(int dimensions, double init) : array(dimensions, vector<double>(dimensions, init)) {}
mat::mat(int rows, int cols, double init) : array(rows, vector<double>(cols, init)) {}
mat::mat(const mat &m) : array(m.array) {}
mat::mat(const vector<vector<double>> &a) : array(a) {}

mat *mat::set(int row, int col, double new_value)
{
    array[row][col] = new_value;
    return this;
}

int mat::rows() const { return array.size(); }
int mat::cols() const { return array[0].size(); }

mat mat::identity(int dimension)
{
    mat m(dimension, 0.0);
    for (int i = 0; i < dimension; ++i)
    {
        m[i][i] = 1.0;
    }
    return m;
}

mat mat::operator+(double scalar) const
{
    mat out(rows(), cols());
    for (int i = 0; i < rows(); i++)
    {
        for (int j = 0; i < cols(); j++)
        {
            out[i][j] = array[i][j] + scalar;
        }
    }
    return out;
}
mat mat::operator+(const mat &m2) const
{
    if (rows() != m2.rows() || cols() != m2.cols())
    {
        __throw_invalid_argument("both matrices must have the same dimensions");
    }
    mat out(rows(), cols());
    for (int i = 0; i < rows(); i++)
    {
        for (int j = 0; j < cols(); j++)
        {
            out[i][j] = array[i][j] + m2[i][j];
        }
    }
    return out;
}
mat mat::operator-(double scalar) const
{
    mat out(rows(), cols());
    for (int i = 0; i < rows(); i++)
    {
        for (int j = 0; j < cols(); j++)
        {
            out[i][j] = array[i][j] - scalar;
        }
    }
    return out;
}
mat mat::operator-(const mat &m2) const
{
    if (rows() != m2.rows() || cols() != m2.cols())
    {
        __throw_invalid_argument("both matrices must have the same dimensions");
    }
    mat out(rows(), cols());
    for (int i = 0; i < rows(); i++)
    {
        for (int j = 0; j < cols(); j++)
        {
            out[i][j] = array[i][j] - m2[i][j];
        }
    }
    return out;
}
mat mat::operator*(double scalar) const
{
    mat out(rows(), cols());
    for (int i = 0; i < rows(); i++)
    {
        for (int j = 0; j < cols(); j++)
        {
            out[i][j] = array[i][j] * scalar;
        }
    }
    return out;
}
mat operator*(double scalar, const mat &m)
{
    mat out = mat(m.rows(), m.cols());
    for (int i = 0; i < m.rows(); i++)
    {
        for (int j = 0; j < m.cols(); j++)
        {
            out[i][j] = m[i][j] * scalar;
        }
    }
    return out;
}
mat mat::operator/(double scalar) const
{
    mat out(rows(), cols());
    for (int i = 0; i < rows(); i++)
    {
        for (int j = 0; j < cols(); j++)
        {
            out[i][j] = array[i][j] / scalar;
        }
    }
    return out;
}

mat mat::operator*(const mat &m2) const
{
    if (cols() != m2.rows())
    {
        __throw_invalid_argument("bad dimensions for matrix multiplication");
    }
    mat out(rows(), m2.cols());
    for (int i = 0; i < rows(); i++)
    {
        for (int k = 0; i < cols(); k++)
        {
            double value_at_k = array[i][k];
            for (int j = 0; j < m2.cols(); j++)
            {
                out[i][j] += value_at_k * m2[k][j];
            }
        }
    }
    return out;
}

vec mat::operator*(const vec &v) const
{
    if (cols() != v.size())
    {
        __throw_invalid_argument("bad dimensions for matrix multiplication");
    }
    vec out(rows());
    for (int i = 0; i < rows(); i++)
    {
        for (int j = 0; j < cols(); j++)
        {
            out[j] += v[j] * array[i][j];
        }
    }
    return out;
}

vector<double> &mat::operator[](int index)
{
    return array[index];
}
const vector<double> &mat::operator[](int index) const { return array[index]; }

mat mat::transpose() const
{
    mat out(cols(), rows());
    for (int i = 0; i < rows(); i++)
    {
        for (int j = 0; j < cols(); j++)
        {
            out[j][i] = array[i][j];
        }
    }
    return out;
}
mat mat::minor(int row_removed, int col_removed) const
{
    mat out(rows() - 1, cols() - 1);
    int minor_row, minor_column = 0;
    for (int i = 0; i < rows(); i++)
    {
        if (i == row_removed)
        {
            continue;
        }
        minor_column = 0;
        for (int j = 0; j < cols(); j++)
        {
            if (j == col_removed)
            {
                continue;
            }
            out[minor_row][minor_column] = array[i][j];
            minor_column++;
        }
        minor_row++;
    }
    return out;
}
double mat::det() const
{
    if (rows() != cols())
    {
        __throw_invalid_argument("matrix must be square");
    }
    if (rows() == 2)
    {
        return array[0][0] * array[1][1] - array[0][1] * array[1][0];
    }
    double out = 0.0;
    for (int i = 0; i < cols(); i++)
    {
        out += pow(-1, i) * array[0][i] * minor(0, i).det();
    }
    return out;
}
mat mat::inverse() const
{
    double determinant = det();
    if (determinant < 1e-9)
    {
        __throw_invalid_argument("matrix isn't invertible");
    }
    mat cofactors(rows(), cols());
    for (int i = 0; i < rows(); i++)
    {
        for (int j = 0; j < cols(); j++)
        {
            cofactors[i][j] = pow(-1, i + j) * minor(i, j).det();
        }
    }
    return cofactors.transpose() / determinant;
}
