#pragma once
#include <cmath>
#include <exception>
#include <string>
#include <vector>
#include <random>
#include <sstream>
#include "vec.h"

using namespace std;

class vec;

class mat
{
public:
    mat();                                      // default
    mat(int dimensions, double init = 0.0);     // square
    mat(int rows, int cols, double init = 0.0); // size
    mat(const mat &m);                          // copy constructor
    mat(const vector<vector<double>> &array);   // initialize from array

    mat *set(int row, int col, double new_value);

    int rows() const;
    int cols() const;

    static mat identity(int dimension); // identity matrix

    mat operator+(double scalar) const;                // scalar addition
    mat operator+(const mat &m2) const;                // element-wise addition
    mat operator-(double scalar) const;                // scalar subtraction
    mat operator-(const mat &m2) const;                // element-wise subrtaction
    mat operator*(double scalar) const;                // scalar multiplication
    friend mat operator*(double scalar, const mat &m); // scalar multiplication
    mat operator/(double scalar) const;                // division by a scalar
    mat operator*(const mat &m2) const;                //  matrix multiplicaton
    vec operator*(const vec &v) const;                 //  matrix multiplicaton

    vector<double> &operator[](int index);             // indexing
    const vector<double> &operator[](int index) const; // constant indexing

    mat transpose() const;
    mat minor(int row_removed, int col_removed) const;
    double det() const;
    mat inverse() const;

    string to_string() const;

private:
    vector<vector<double>> array;
};
