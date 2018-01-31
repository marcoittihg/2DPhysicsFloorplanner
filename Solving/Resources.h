//
// Created by Marco on 25/01/18.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_RESOURCES_H
#define BUBBLEREGIONSFLOORPLANNER_RESOURCES_H

#include "../Data/FPGAData/Block.h"

struct Resources{
public:
    unsigned short CLB = 0;
    unsigned short BRAM = 0;
    unsigned short DSP = 0;
    unsigned short FORBIDDEN = 0;

    void addBlock(Block block){
        switch (block){
            case Block ::CLB_BLOCK: CLB++;
                break;
            case Block ::DSP_BLOCK: DSP++;
                break;
            case Block ::BRAM_BLOCK: BRAM++;
                break;
            case Block ::FORBIDDEN_BLOCK: FORBIDDEN++;
                break;
        }
    }

    void add(Resources res){
        CLB += res.CLB;
        BRAM += res.BRAM;
        DSP += res.DSP;
        FORBIDDEN += res.FORBIDDEN;
    }

    void sub(Resources res){
        CLB -= res.CLB;
        BRAM -= res.BRAM;
        DSP -= res.DSP;
        FORBIDDEN -= res.FORBIDDEN;
    }
};


#endif //BUBBLEREGIONSFLOORPLANNER_RESOURCES_H
