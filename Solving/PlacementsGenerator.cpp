#include "PlacementsGenerator.h"

PlacementsGenerator::PlacementsGenerator(Problem* problem) {
	if (problem == NULL)
		throw std::invalid_argument("Given problem pointer isn't valid.");

	this->_problem = problem;
}

PlacementsGenerator::~PlacementsGenerator() {
	delete[] this->_maxPlacementArea;
}

std::vector<std::vector<FeasiblePlacement>>* PlacementsGenerator::getFeasiblePlacements(float excess) {

	uint8_t maxX = this->_problem->getBoard()->getDimension().get_x();
	int totalRegions = this->_problem->getNumRegions();

	std::list<FeasiblePlacement*>* feasiblePlacementsList = new std::list<FeasiblePlacement*>[totalRegions];
	this->_maxPlacementArea = new int[totalRegions];
	std::fill(this->_maxPlacementArea, this->_maxPlacementArea + totalRegions, std::numeric_limits<int>::max());

	for (int width = 1; width < maxX; width++) {
		for (int startX = 0; startX + width <= maxX; startX++) {

			std::list<FeasiblePlacement*>* currentColumn = this->getPlacements(startX, width);

			for (int i = 0; i < totalRegions; i++) {

				if (currentColumn[i].empty()) continue;
				
				// Update current best area if necessary
				int currentMaxArea = (*(currentColumn[i].begin()))->getArea() * excess;
				if (currentMaxArea < this->_maxPlacementArea[i])
					this->_maxPlacementArea[i] = currentMaxArea;

				// Fill global placement list with newly found placements
				feasiblePlacementsList[i].splice(feasiblePlacementsList[i].end(), currentColumn[i]);
			}

			delete[] currentColumn;
		}
	}

	// TODO: remove useless from feasiblePlacementsList

	std::vector<std::vector<FeasiblePlacement>>* feasiblePlacements = new std::vector<std::vector<FeasiblePlacement>>();
	feasiblePlacements->resize(totalRegions);

	for (int i = 0; i < totalRegions; i++) {
		for (auto it = feasiblePlacementsList[i].begin(); it != feasiblePlacementsList[i].end(); ++it)
			feasiblePlacements->at(i).push_back(**it);
	}

	delete[] feasiblePlacementsList;

	return feasiblePlacements;
}

std::list<FeasiblePlacement*>* PlacementsGenerator::getPlacements(uint8_t startX, uint8_t width) {

	uint8_t maxHeight = this->_problem->getBoard()->getDimension().get_y();
	uint8_t tileHeight = this->_problem->getBoard()->getTileHeight();
	
	// Check if PR region could be placed here
	bool canPlacePR = this->_problem->getLeftValidIDs(startX)
		&& this->_problem->getRightValidIDs(startX + width - 1);

	std::list<const ProblemRegion*> placeableRegions;

	// Fill up placeable regions list for current X values
	for (int r = this->_problem->getNumRegions() - 1; r >= 0; --r) {

		const ProblemRegion* currentRegion = this->_problem->getFloorplanProblemRegion(r);

		// Check if current region is PR and remove it if can't be placed with this horizontal bounds
		if (currentRegion->getType() == RegionType::PR && !canPlacePR) continue;

		placeableRegions.push_back(currentRegion);
	}

	std::list<FeasiblePlacement*>* feasiblePlacements = new std::list<FeasiblePlacement*>[this->_problem->getNumRegions()];

	// If there are no placemnts which can fit in this area
	// return empty
	if (placeableRegions.empty()) return feasiblePlacements;

	Point2D start(startX, 0), dimension(width, 1);
	
	// Check all available placements heights rising from the shortest
	for (int height = 1; height < maxHeight; height++) {

		dimension.set_y(height);

		int currentArea = dimension.get_x() * dimension.get_y();

		for (int startY = 0; startY + height <= maxHeight; startY++) {

			start.set_y(startY);

			// Get current resources
			std::map<Block, int> currentResources = 
				this->_problem->getBoard()->getResourcesFor(start, dimension);

			// If current placement is forbidden continue
			if (currentResources.at(Block::FORBIDDEN_BLOCK) > 0) continue;

			// Calculate current resources cost
			int currentCost = currentResources.at(Block::CLB_BLOCK) * this->_problem->getCLBCost()
				+ currentResources.at(Block::DSP_BLOCK) * this->_problem->getDSPCost()
				+ currentResources.at(Block::BRAM_BLOCK) * this->_problem->getBRAMCost();

			// Check validity for each region
			for (auto it = placeableRegions.begin(); it != placeableRegions.end(); ++it) {

				// If current area doesn't exceed requestd percentage compared to minimum found placement
				// and resources are sufficient, store current placement for this region
				if (currentArea < this->_maxPlacementArea[(*it)->getID()] 
					&& (*it)->isContained(&currentResources))
					feasiblePlacements[(*it)->getID()]
					.emplace_back(new FeasiblePlacement(start, dimension, (*it)->getType(), currentCost));
			}

		}
	}
	
	return feasiblePlacements;
}