#include "PlacementsGenerator.h"

using namespace std;

PlacementsGenerator::PlacementsGenerator(Problem* problem) {
	if (problem == NULL)
		throw invalid_argument("Given problem pointer isn't valid.");

	this->_problem = problem;
}

PlacementsGenerator::~PlacementsGenerator() {
	delete[] this->_maxPlacementArea;
}

vector<vector<FeasiblePlacement>>* PlacementsGenerator::getFeasiblePlacements(float excess, int minWidth, int minHeight) {

	uint8_t maxX = this->_problem->getBoard()->getDimension().get_x();
	int totalRegions = this->_problem->getNumRegions();

	list<FeasiblePlacement*>* feasiblePlacementsList = new list<FeasiblePlacement*>[totalRegions];
	this->_maxPlacementArea = new int[totalRegions];
	fill(this->_maxPlacementArea, this->_maxPlacementArea + totalRegions, numeric_limits<int>::max());

	for (int width = minWidth; width < maxX; width++) {
		for (int startX = 0; startX + width <= maxX; startX++) {

			list<FeasiblePlacement*>* currentColumn = this->getPlacements(startX, width, minHeight);

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
	
	vector<vector<FeasiblePlacement>>* feasiblePlacements = new vector<vector<FeasiblePlacement>>();
	feasiblePlacements->resize(totalRegions);

	int maxP = 0, minP = numeric_limits<int>::max(), allP = 0;

	for (int i = 0; i < totalRegions; i++) {
		for (auto it = feasiblePlacementsList[i].begin(); it != feasiblePlacementsList[i].end(); ++it) {
			if((*it)->getArea() <= this->_maxPlacementArea[i])
				feasiblePlacements->at(i).push_back(**it);
		}

		int p = feasiblePlacements->at(i).size();

		if (p > maxP)
			maxP = p;

		if (p < minP)
			minP = p;

		allP += p;

		cout << "Region " << i << " has " << p << " placements." << endl;

		while (!feasiblePlacementsList[i].empty())
		{
			FeasiblePlacement* fp = feasiblePlacementsList[i].back();
			feasiblePlacementsList[i].pop_back();

			delete fp;
		}
	}

	delete[] feasiblePlacementsList;

	cout << "Minimum placements per region: " << minP << endl;
	cout << "Maximum placements per region: " << maxP << endl;
	cout << "Avarage placements per region: " << allP / feasiblePlacements->size() << endl;

	return feasiblePlacements;
}

list<FeasiblePlacement*>* PlacementsGenerator::getPlacements(uint8_t startX, uint8_t width, int minHeight) {

	uint8_t maxHeight = this->_problem->getBoard()->getDimension().get_y();
	uint8_t tileHeight = this->_problem->getBoard()->getTileHeight();
	
	// Check if PR region could be placed here
	bool canPlacePR = this->_problem->getLeftValidIDs(startX)
		&& this->_problem->getRightValidIDs(startX + width - 1);

	list<const ProblemRegion*> placeableRegions;

	// Fill up placeable regions list for current X values
	for (int r = this->_problem->getNumRegions() - 1; r >= 0; --r) {

		const ProblemRegion* currentRegion = this->_problem->getFloorplanProblemRegion(r);

		// Check if current region is PR and remove it if can't be placed with this horizontal bounds
		if (currentRegion->getType() == RegionType::PR && !canPlacePR) continue;

		placeableRegions.push_back(currentRegion);
	}

	list<FeasiblePlacement*>* feasiblePlacements = new list<FeasiblePlacement*>[this->_problem->getNumRegions()];

	// If there are no placemnts which can fit in this area
	// return empty
	if (placeableRegions.empty()) return feasiblePlacements;

	Point2D start(startX, 0), dimension(width, 1);
		
	// Check all available placements heights rising from the shortest
	for (int height = minHeight; height < maxHeight; height++) {

		dimension.set_y(height);

		int currentArea = dimension.get_x() * dimension.get_y();

		for (int startY = 0; startY + height <= maxHeight; startY++) {

			start.set_y(startY);

			// Get current resources
			map<Block, int> currentResources = 
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

		// If placement was found with last height, avoid further resources
		placeableRegions.remove_if([feasiblePlacements](const ProblemRegion* r) { return !feasiblePlacements[r->getID()].empty(); });

		if (placeableRegions.empty()) break;
	}
	
	return feasiblePlacements;
}