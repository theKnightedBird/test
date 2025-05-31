#include "vec.h"

using namespace std;

vec::vec() {}
vec::vec(int size) : array(size, 0) {}
vec::vec(const vec &v) : array(v.array) {}
vec::vec(const vector<double> &array) : array(array) {}

vec *vec::set(int index, double new_value)
{
    array[index] = new_value;
    return this;
}

double vec::size() const { return array.size(); }

vec vec::operator+(double scalar) const
{
    vec out(size());
    for (int i = 0; i < size(); i++)
    {
        out[i] = array[i] + scalar;
    }
    return out;
}
vec vec::operator+(const vec &v2) const
{
    vec out(size());
    for (int i = 0; i < size(); i++)
    {
        out[i] = array[i] + v2[i];
    }
    return out;
}
vec vec::operator-(double scalar) const
{
    vec out(size());
    for (int i = 0; i < size(); i++)
    {
        out[i] = array[i] - scalar;
    }
    return out;
}
vec vec::operator-(const vec &v2) const
{
    vec out(size());
    for (int i = 0; i < size(); i++)
    {
        out[i] = array[i] - v2[i];
    }
    return out;
}
vec vec::operator*(double scalar) const
{
    vec out(size());
    for (int i = 0; i < size(); i++)
    {
        out[i] = array[i] * scalar;
    }
    return out;
}
vec operator*(double scalar, const vec &v)
{
    vec out(v.size());
    for (int i = 0; i < v.size(); i++)
    {
        out[i] = v[i] * scalar;
    }
    return out;
    ;
}
vec vec::operator/(double scalar) const
{
    vec out(size());
    for (int i = 0; i < size(); i++)
    {
        out[i] = array[i] / scalar;
    }
    return out;
}

double vec::operator*(const vec &v2) const
{
    double out = 0;
    for (int i = 0; i < size(); i++)
    {
        out += array[i] * v2[i];
    }
    return out;
}

double &vec::operator[](int index)
{
    return array[index];
}

const double &vec::operator[](int index) const
{
    return array[index];
}

mat vec::to_col_matrix() const
{
    mat out = mat(size(), 1);
    for (int i = 0; i < size(); i++)
    {
        out[i][0] = array[i];
    }
    return mat();
}

mat vec::to_row_matrix() const
{
    mat out = mat(1, size());
    for (int i = 0; i < size(); i++)
    {
        out[0][i] = array[i];
    }
    return mat();
}

mat vec::to_diagonal_matrix() const
{
    mat out = mat(size());
    for (int i = 0; i < size(); i++)
    {
        out[i][i] = array[i];
    }
    return mat();
}

double vec::mag_squared() const
{
    double out = 0;
    for (double i : array)
    {
        out += pow(i, 2);
    }
    return out;
}

double vec::mag() const
{
    return sqrt(mag_squared());
}

vec vec::normalized() const
{
    return *this / mag();
}

vec vec::proj(const vec &projecting, const vec &onto)
{
    return (projecting * onto / onto.mag()) * onto;
}

double vec::dist_between(const vec &v1, const vec &v2)
{
    double out;
    for (int i = 0; i < v1.size(); i++)
    {
        out += pow(v1[i] - v2[i], 2);
    }
    return sqrt(out);
}
