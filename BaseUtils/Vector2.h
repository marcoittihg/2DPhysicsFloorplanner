//
// Created by Marco on 28/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_VECTOR2_H
#define BUBBLEREGIONSFLOORPLANNER_VECTOR2_H

/** 2D float vector
 */
class Vector2 {

private:
    float x;
    float y;

public:

    Vector2();

    Vector2(float x, float y);

    /**
     * Add to the vector the component of the passed vector
     * @param v2
     */
    void add(Vector2 v2);

    /**
     * Multiply all the components of the vector by c
     * @param c
     */
    void multiply(float c);


    float getX() const;

    void setX(float x);

    float getY() const;

    void setY(float y);

    /**
     * Dot product
     * @param v2
     * @return
     */
    float dot(Vector2 v2);

    float mangnitude();

    float sqrMagnitude();

    void normalize();
};


#endif //BUBBLEREGIONSFLOORPLANNER_VECTOR2_H
