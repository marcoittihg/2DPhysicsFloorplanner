#ifndef BUBBLEREGIONSFLOORPLANNER_PLACEMENTS_GENERATOR_H
#define BUBBLEREGIONSFLOORPLANNER_PLACEMENTS_GENERATOR_H

#include "../Data/ProblemData/Problem.h"
#include "FeasiblePlacement.h"
#include <vector>
#include <list>
#include <cstdint>
#include <iostream>
#include <functional>

class PlacementsGenerator {

public:
	PlacementsGenerator(Problem* problem);
	virtual ~PlacementsGenerator();

	std::vector<std::vector<FeasiblePlacement>>* getFeasiblePlacements(float excess = 1.00f, int minWidth = 1, int minHeight = 1);

private:
	Problem * _problem;
	int* _maxPlacementArea;

	std::list<FeasiblePlacement*>* getPlacements(uint8_t startX, uint8_t width, int minHeight);
};

#endif // BUBBLEREGIONSFLOORPLANNER_PLACEMENTS_GENERATOR_H