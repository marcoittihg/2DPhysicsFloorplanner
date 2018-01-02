//
// Created by Marco on 28/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_FEASIBLEPLACEMENTSTATE_H
#define BUBBLEREGIONSFLOORPLANNER_FEASIBLEPLACEMENTSTATE_H


#include <cstdint>

/**
 * Feasible state of a possible placement of a region
 */
enum FeasiblePlacementState : uint8_t {
    PLACEMENT_AVAILABLE,
    PLACEMENT_TAKEN,
    PLACEMENT_FORBIDDEN
};


#endif //BUBBLEREGIONSFLOORPLANNER_FEASIBLEPLACEMENTSTATE_H
