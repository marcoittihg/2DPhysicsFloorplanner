//
// Created by Marco on 31/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_FLOORPLANNINGMANAGER_H
#define BUBBLEREGIONSFLOORPLANNER_FLOORPLANNINGMANAGER_H


#include "MainLoopManager.h"
#include "../Data/ProblemData/Problem.h"
#include "FloortplanningMangerState.h"
#include "../BaseUtils/Vector2.h"
#include "FloorplanSolution.h"

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

    void setStartTime(time_t startTime);

private:
    Problem* problem;

    float _time;

    FloortplanningMangerState state;

    float wireStabTime;

    float wireStabImprovementTime;

    float closestAlternativeRefreshTime;

    float lastAlternativeRefreshTime;

    float placemStabTime;

    time_t tmpTime;

    Vector2* oldWireStabRegionPos;

    float oldWireStabWirelen;

    float realWireStabPositionsImprovementTime;

    time_t phaseStartTime;

    time_t startTime;

    FloorplanSolution* solutions;

    int numSolutions = 30;

    int bestSolutionScore;

    float minDisplacePercentage;
    float maxDisplacePercentage;
    int cont = 0;

};


#endif //BUBBLEREGIONSFLOORPLANNER_FLOORPLANNINGMANAGER_H
