//
// Created by Marco on 28/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_RIGIDBODY_H
#define BUBBLEREGIONSFLOORPLANNER_RIGIDBODY_H


#include "../../BaseUtils/Vector2.h"
#include "Physics.h"


class Rigidbody {
    /** Actual position of the region
     */
    Vector2 position;

    /** Actual speed of the region
     */
    Vector2 speed;

    /** Actual dimension of the region
     */
    Vector2 dimension;

    /** Force applied for this step
     */
    Vector2 stepForce;
public:
    Rigidbody();


    const Vector2 &getPosition() const;

    void setPosition(const Vector2 &position);

    const Vector2 &getSpeed() const;

    void setSpeed(const Vector2 &speed);

    const Vector2 &getDimension() const;

    void setDimension(const Vector2 &dimension);

    /** Add an impulsive force to the rigidbody
     * @param force The force to apply
     */
    void addImpulse(Vector2 force);

    /**Moves the region for a time step of length dt and apply the linear drag
     * @param dt
     */
    void applyMovement(float dt);

    void applyForces(float dt);

    void applyLinearDrag(float linearDrag);

    void resetStepForce();

    const Vector2 &getStepForce() const;
};


#endif //BUBBLEREGIONSFLOORPLANNER_RIGIDBODY_H
