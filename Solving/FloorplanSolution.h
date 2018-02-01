//
// Created by Marco on 31/01/18.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_FLOORPLANSOLUTION_H
#define BUBBLEREGIONSFLOORPLANNER_FLOORPLANSOLUTION_H


#include "FeasiblePlacement.h"

struct FloorplanSolution {
    int* placementsIndexes;
    int score;
};


#endif //BUBBLEREGIONSFLOORPLANNER_FLOORPLANSOLUTION_H
