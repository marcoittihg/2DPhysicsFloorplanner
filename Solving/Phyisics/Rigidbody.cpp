//
// Created by Marco on 28/12/17.
//

#include "Rigidbody.h"

const Vector2 &Rigidbody::getPosition() const {
    return position;
}

void Rigidbody::setPosition(const Vector2 &position) {
    Rigidbody::position = position;
}

const Vector2 &Rigidbody::getSpeed() const {
    return speed;
}

void Rigidbody::setSpeed(const Vector2 &speed) {
    Rigidbody::speed = speed;
}

const Vector2 &Rigidbody::getDimension() const {
    return dimension;
}

void Rigidbody::setDimension(const Vector2 &dimension) {
    Rigidbody::dimension = dimension;
    safeDist = dimension.getX() * dimension.getX() + dimension.getY() * dimension.getY();
}

void Rigidbody::applyMovement(float dt, float linearDrag) {
    speed.multiply(1 - linearDrag);
    Vector2 tmp = speed;
    tmp.multiply(dt);
    position.add(tmp);
}

void Rigidbody::addImpulse(Vector2 force, float dt) {
    force.multiply(dt/mass);
    speed.add(force);
}


Rigidbody::Rigidbody() {
    this->speed = Vector2(0,0);
    this->dimension = Vector2(0,0);
    this->position = Vector2(0,0);
    this->mass = 1;
}

float Rigidbody::getSafeDist() const {
    return safeDist;
}

void Rigidbody::setMass(float mass) {
    Rigidbody::mass = mass;
}

float Rigidbody::getMass() const {
    return mass;
}
