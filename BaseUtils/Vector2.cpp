//
// Created by Marco on 28/12/17.
//

#include <cmath>
#include "Vector2.h"

float Vector2::getX() const {
    return x;
}

void Vector2::setX(float x) {
    Vector2::x = x;
}

float Vector2::getY() const {
    return y;
}

void Vector2::setY(float y) {
    Vector2::y = y;
}

void Vector2::add(Vector2 v2) {
    x += v2.x;
    y += v2.y;
}

void Vector2::multiply(float c) {
    x *= c;
    y *= c;
}

Vector2::Vector2() {
    x = y = 0;
}

float Vector2::dot(Vector2 v2) {
    return x * v2.x + y * v2.y;
}

Vector2::Vector2(float x, float y) : x(x), y(y) {}

float Vector2::sqrMagnitude() {
    return x*x+y*y;
}

float Vector2::mangnitude() {
    return sqrt(sqrMagnitude());
}

void Vector2::normalize() {
    float mag = mangnitude();
    x /= mag;
    y /= mag;
}

