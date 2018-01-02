//
// Created by Marco on 31/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_FLOORPLANNINGMANAGER_H
#define BUBBLEREGIONSFLOORPLANNER_FLOORPLANNINGMANAGER_H


#include "MainLoopManager.h"
#include "../Data/ProblemData/Problem.h"
#include "FloortplanningMangerState.h"

/**
 * Manages the floorplanning phases
 */
class FloorplanningManager {
public:
    static FloorplanningManager& getINSTANCE(){
        static FloorplanningManager floorplanningManager;
        return floorplanningManager;
    }

    static constexpr float maxSeparationCoeff = 10;

    void start();

    void onPysicsStep();

    Problem *getProblem() const;

    void setProblem(Problem *problem);

    FloortplanningMangerState getState() const;

private:
    Problem* problem;

    float time;

    FloortplanningMangerState state;

    float wireStabTime;

    float closestAlternativeRefreshTime;

    float lastAlternativeRefreshTime;

    float placemStabTime;
};


#endif //BUBBLEREGIONSFLOORPLANNER_FLOORPLANNINGMANAGER_H
