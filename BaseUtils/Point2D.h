//
// Created by Marco on 28/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_POINT2D_H
#define BUBBLEREGIONSFLOORPLANNER_POINT2D_H


/**
 * 2D coordinate data
 */
class Point2D{
private:
    unsigned char _x;
    unsigned char _y;

public:
    /**
     * Constructor
     * @param _x
     * @param _y
     */
    Point2D(unsigned char _x, unsigned char _y);

    Point2D();

    /*
     * Getters and setters
     */
    unsigned char get_x() const;

    void set_x(unsigned char _x);

    unsigned char get_y() const;

    void set_y(unsigned char _y);
};


#endif //BUBBLEREGIONSFLOORPLANNER_POINT2D_H
