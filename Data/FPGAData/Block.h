//
// Created by Marco on 28/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_BLOCK_H
#define BUBBLEREGIONSFLOORPLANNER_BLOCK_H

#include <cstdint>

enum Block : uint8_t {
    NULL_BLOCK = '-',
    CLB_BLOCK = 'C',
    BRAM_BLOCK = 'B',
    DSP_BLOCK = 'D',
    FORBIDDEN_BLOCK = 'F'
};




#endif //BUBBLEREGIONSFLOORPLANNER_BLOCK_H
