/*
 *  Copyright Pentrek Inc, 2022
 */

#ifndef _pentrek_point_h_
#define _pentrek_point_h_

#include "include/pentrek_types.h"

namespace pentrek {

struct IPoint {
    int32_t x, y;
    
    bool operator==(const IPoint o) const {
        return x == o.x && y == o.y;
    }
    bool operator!=(const IPoint o) const { return !(*this == o); }
    
    IPoint operator-() const { return {-x, -y}; }

    IPoint operator+(IPoint p) const {
        return {x + p.x, y + p.y};
    }
    IPoint operator-(IPoint p) const {
        return {x - p.x, y - p.y};
    }
};

struct Point {
    float x, y;
    
    float* floats() { return &x; }
    const float* floats() const { return &x; }
    
    Point operator-() const { return {-x, -y}; }

    bool operator==(const Point o) const {
        return x == o.x && y == o.y;
    }
    bool operator!=(const Point o) const { return !(*this == o); }

    Point operator+(Point p) const {
        return {x + p.x, y + p.y};
    }
    Point operator-(Point p) const {
        return {x - p.x, y - p.y};
    }
    Point operator*(Point p) const {
        return {x * p.x, y * p.y};
    }

    friend Point operator*(Point p, float s) {
        return {p.x * s, p.y * s};
    }
    friend Point operator*(float s, Point p) { return p * s; }

    friend Point operator/(Point p, float s) {
        return {p.x / s, p.y / s};
    }

    Point& operator+=(Point p) {
        *this = *this + p;
        return *this;
    }
    Point& operator-=(Point p) {
        *this = *this - p;
        return *this;
    }
    Point& operator*=(Point p) {
        *this = *this * p;
        return *this;
    }
    Point& operator*=(float s) {
        *this = *this * s;
        return *this;
    }
    
    bool isZero() const { return x == 0 && y == 0; }

    float lengthSquared() const { return x * x + y * y; }
    float length() const { return std::sqrt(this->lengthSquared()); }
    
    Point cw() const  { return { y, -x}; }
    Point ccw() const { return {-y,  x}; }
    Point normalize() const { return this->makeLength(1); }
    Point makeLength(float length) const;
    
    static float Dot(Point a, Point b) {
        return a.x * b.x + a.y * b.y;
    }
    static float Cross(Point a, Point b) {
        return a.x * b.y - a.y * b.x;
    }
};

} // namespace

#endif
