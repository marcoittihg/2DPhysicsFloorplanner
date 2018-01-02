//
// Created by Marco on 30/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_PHYSICSREGIONTYPE_H
#define BUBBLEREGIONSFLOORPLANNER_PHYSICSREGIONTYPE_H

#include <cstdint>

/**
 * IO_INT Region with IO and interconnections
 * IO_NOINT Region with IO but no interconnections
 * NOIO_INT Region without IO but with interconnections
 * NOIO_NOINT Region without both IO and interconnections
 */
enum PhysicsRegionType : uint8_t {
    IO_INT,
    IO_NOINT,
    NOIO_INT,
    NOIO_NOINT
};


#endif //BUBBLEREGIONSFLOORPLANNER_PHYSICSREGIONTYPE_H
