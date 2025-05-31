#pragma once
#include <cmath>
#include <exception>
#include <string>
#include <vector>
#include <random>
#include <sstream>
#include "mat.h"

class mat;

using namespace std;

class vec
{
public:
    vec();
    vec(int size);
    vec(const vec &v);
    vec(const vector<double> &array);

    vec *set(int index, double new_value);

    double size() const;

    vec operator+(double scalar) const;                // scalar addition
    vec operator+(const vec &v2) const;                // element-wise addition
    vec operator-(double scalar) const;                // scalar subtraction
    vec operator-(const vec &v2) const;                // element-wise subrtaction
    vec operator*(double scalar) const;                // scalar multiplication
    friend vec operator*(double scalar, const vec &v); // scalar multiplication
    vec operator/(double scalar) const;                // division by a scalar
    double operator*(const vec &v2) const;             // dot product

    double &operator[](int index);             // indexing
    const double &operator[](int index) const; // constant indexing

    // vector specific functions

    mat to_col_matrix() const;
    mat to_row_matrix() const;
    mat to_diagonal_matrix() const;
    double mag_squared() const;
    double mag() const;
    vec normalized() const;

    static vec proj(const vec &projecting, const vec &onto);
    static double dist_between(const vec &v1, const vec &v2);

private:
    vector<double> array;
};
