#include <iostream>
#include "Solving/PhysicsRegion.h"
#include "Data/ProblemData/Problem.h"
#include "FileManagement/FileManager.h"
#include "Solving/MainLoopManager.h"
#include "Solving/FloorplanningManager.h"
#include "Solving/PlacementsGenerator.h"
#include <list>
#include <cmath>
#include <ctime>
#include <map>

using namespace std;

float print_resource_usage(const Problem* problem) {

	Point2D origin(0, 0);
	const std::map<Block, int> totalResources = problem->getBoard()->getResourcesFor(origin, problem->getBoard()->getDimension());

	int boardBlockCounter = totalResources.at(Block::CLB_BLOCK)
		+ totalResources.at(Block::DSP_BLOCK)
		+ totalResources.at(Block::BRAM_BLOCK);

	int problemBlockCounter = 0;

	for (int i = 0; i < problem->getNumRegions(); ++i) {
		ProblemRegion* pr = const_cast<ProblemRegion *>(problem->getFloorplanProblemRegion(i));
		problemBlockCounter += pr->getCLBNum() + pr->getDSPNum() + pr->getBRAMNum();
	}

	float percentage = ((float)problemBlockCounter / (float)boardBlockCounter);
	std::cout << "Density: " << percentage * 100 << "%" << std::endl;

	return percentage;
}

int main(int argc, char* argv[]) {
    time_t seconds;
    seconds = time (NULL);

    cout << "*------* STARTING *------*" << endl;
    cout << "Loading data problem from file\n" << endl;

    Problem* problem;
    try {
        problem = FileManager::getINSTANCE().readProblem("F:/Documenti/Visual Studio 2017/Projects/BubbleRegionsFloorplanner/Problems/10021");
    }catch ( const std::invalid_argument& e ){
        fprintf(stderr, e.what());
    }
	
	float percentage = print_resource_usage(problem);

	//float maxArea = 2*std::exp(-problem->getNumRegions()/20)+1.05;
	float maxArea = 1.0 / 3.0 / percentage + 2.0 / 3.0 + 0.05;
	std::cout << "Problem max area: " << maxArea << std::endl;

	PlacementsGenerator* pg = new PlacementsGenerator(problem);

    std::vector<std::vector<FeasiblePlacement>> feasiblePlacements = *(pg->getFeasiblePlacements(maxArea));

	delete pg;

	int maxP = 0, minP = std::numeric_limits<int>::max(), allP = 0;

	for (int rID = 0; rID < feasiblePlacements.size(); rID++) {
		int p = feasiblePlacements.at(rID).size();

		if (p > maxP)
			maxP = p;

		if (p < minP)
			minP = p;

		allP += p;

		cout << "Region " << rID << " has " << p << " feasible placements." << endl;
	}

	cout << "Minimum placements per region: " << minP << endl;
	cout << "Maximum placements per region: " << maxP << endl;
	cout << "Avarage placements per region: " << allP / feasiblePlacements.size() << endl;

    time_t seconds2;
    seconds2 = time (NULL);
	std::cout << "Feasible placements search time : " << seconds2 - seconds << std::endl;
    seconds = seconds2;

    //Precalculate resorces of each placement
    for (int i = 0; i < feasiblePlacements.size(); ++i) {
        std::vector<FeasiblePlacement> placementVector = feasiblePlacements.at(i);

        for (int j = 0; j < placementVector.size(); ++j) {
            placementVector.at(j).calculateResources(problem->getBoard());
        }
    }

    //Create regions
    PhysicsRegion* region;
    for (int i = 0; i < feasiblePlacements.size(); ++i) {
        std::vector<FeasiblePlacement> placementVector = feasiblePlacements.at(i);
        FeasiblePlacement* placementArray = new FeasiblePlacement[placementVector.size()];

        for (int j = 0; j < placementVector.size(); ++j) {
            placementArray[j] = placementVector.at(j);
        }

        region = Physics::getINSTANCE().addRegion();
        region->setRegionIndex(i);
        region->setRegionState(PhysicsRegionState::FLOATING);

        region->setPlacementNum(placementVector.size());
        region->setFeasiblePlacements(placementArray);
        region->setRegionIO(problem->getFloorplanProblemRegion(i)->getRegionIO());
        region->setIONum(problem->getFloorplanProblemRegion(i)->getIONum());

    }

    PhysicsRegion* regions = Physics::getINSTANCE().getPhysicsRegions();
    int numRegions = Physics::getINSTANCE().getRegionNum();
    //Link all the interconnected regions
    for (int i = 0; i < numRegions; ++i) {
        for (int j = i + 1; j < numRegions; ++j) {
            int numWire = std::fmax(problem->getInterconnectionsMatrix(i,j), problem->getInterconnectionsMatrix(j,i) );

            if(numWire <= 0)
                continue;

            regions[i].addInterconnectedRegion(&regions[j], numWire);
            regions[j].addInterconnectedRegion(&regions[i], numWire);
        }
    }

    //classify all the regions
    for (int i = 0; i < numRegions; ++i) {
        PhysicsRegion* region1 = &regions[i];

        if(problem->getWireCost() == 0){
            //If the wire cost do not count each region
            // is considered ad a region without connections
            region1->setRegionType(PhysicsRegionType::NOIO_NOINT);
        }

        if(region1->getIONum() > 0){
            if(region1->getInterconnectedRegions().size() > 0){
                //Region with io and inderconnections
                region1->setRegionType(PhysicsRegionType::IO_INT);
            }else{
                //Region with IO but not interconnections
                region1->setRegionType(PhysicsRegionType::IO_NOINT);
            }
        }else{
            if(region1->getInterconnectedRegions().size() > 0){
                //Region without Io but with interconnections
                region1->setRegionType(PhysicsRegionType::NOIO_INT);
            }else{
                //Region without IO and without interconnections
                //Consider only the area and if the area weight is 0 try to take the smallest placement
                region1->setRegionType(PhysicsRegionType::NOIO_NOINT);
            }
        }
    }


    seconds2 = time (NULL);
    std::cout<<"Regions elimination time : "<<seconds2 - seconds<<std::endl;

    Physics::getINSTANCE().setBoard(problem->getBoard());


    FloorplanningManager::getINSTANCE().setProblem(problem);
    FloorplanningManager::getINSTANCE().start();

    MainLoopManager::getINSTANCE().startLoop();

    return 0;
}
