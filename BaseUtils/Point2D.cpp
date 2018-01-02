//
// Created by Marco on 28/12/17.
//

#include "Point2D.h"

Point2D::Point2D(unsigned char _x, unsigned char _y) : _x(_x), _y(_y) {
}

unsigned char Point2D::get_x() const {
    return _x;
}

void Point2D::set_x(unsigned char _x) {
    Point2D::_x = _x;
}

unsigned char Point2D::get_y() const {
    return _y;
}

void Point2D::set_y(unsigned char _y) {
    Point2D::_y = _y;
}

Point2D::Point2D() {
    _x = 0;
    _y = 0;
}