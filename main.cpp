#include <iostream>
#include "Solving/PhysicsRegion.h"
#include "Solving/Render.h"
#include "Data/ProblemData/Problem.h"
#include "FileManagement/FileManager.h"
#include "Solving/MainLoopManager.h"
#include "Solving/FloorplanningManager.h"
#include <GLFW/glfw3.h>
#include <unistd.h>
#include <list>
#include <cmath>

using namespace std;

void getAllFeasiblePlacements(std::vector<std::vector<FeasiblePlacement>>* feasiblePlacements, Problem* problem){
    std::cout<<"Generation feasible solutions"<<std::endl;

    char tileHeight = problem->getBoard()->getTileHeight();

    //Linked list that contains all the feasible placements found
    std::list<FeasiblePlacement>* feasiblePlacementsArray = new std::list<FeasiblePlacement>[problem->getNumRegions()];

    for (int i = 0; i < problem->getNumRegions(); ++i) {
        std::cout<<"Region: "<<i<<std::endl;

        //For each region of the problem
        ProblemRegion* problemRegion = const_cast<ProblemRegion *>(problem->getFloorplanProblemRegion(i));
        RegionType  regionType = problemRegion->getType();

        for (int j = 0; j < problem->getBoard()->getDimension().get_y(); ++j) {
            for (int k = j + 1; k < problem->getBoard()->getDimension().get_y(); ++k) {
                //For each possible couple of indexes

                if(regionType == RegionType::PR && ( problem->getLeftValidIDs(j) == 0 || problem->getRightValidIDs(k) == 0 ))
                    continue;   //The region is PR and (j,k) are not feasible start and end points for that region

                for (int h = 1; h <= problem->getBoard()->getDimension().get_x(); ++h) {
                    //For each possible height of the board


                    int CLBNum, BRAMNum, DSPNum, FORBBlock;
                    CLBNum = BRAMNum = DSPNum = FORBBlock = 0;
                    for (int t = 0; t <= problem->getBoard()->getDimension().get_x() - h; ++t) {
                        //Check for each possible translation of the area

                        //Calculate the number of blocks inside the area
                        // (from index [j] to index [k] starting from height [l] to height [l+h])

                        if(t == 0) {
                            //If that is the first iteration sum all the blocks
                            for (int m = t; m < t + h; ++m) {
                                for (int n = j; n <= k; ++n) {
                                    Block block = problem->getBoard()->getBlockMatrix(m, n);

                                    switch (block) {
                                        case Block::CLB_BLOCK :
                                            CLBNum++;
                                            break;
                                        case Block::BRAM_BLOCK :
                                            BRAMNum++;
                                            break;
                                        case Block::DSP_BLOCK :
                                            DSPNum++;
                                            break;
                                        case Block::FORBIDDEN_BLOCK :
                                            FORBBlock++;
                                            break;
                                    }
                                }
                            }
                        }else{
                            //If that's not the first iteration just subtract the first line and add the last one
                            for (int n = j; n <= k ; ++n) {
                                //Subtract the first one
                                Block block = problem->getBoard()->getBlockMatrix(t - 1, n);

                                switch (block) {
                                    case Block::CLB_BLOCK :
                                        CLBNum--;
                                        break;
                                    case Block::BRAM_BLOCK :
                                        BRAMNum--;
                                        break;
                                    case Block::DSP_BLOCK :
                                        DSPNum--;
                                        break;
                                    case Block::FORBIDDEN_BLOCK :
                                        FORBBlock--;
                                        break;
                                }
                            }

                            for (int n = j; n <= k ; ++n) {
                                //Add the last one
                                Block block = problem->getBoard()->getBlockMatrix(t + h - 1, n);

                                switch (block) {
                                    case Block::CLB_BLOCK :
                                        CLBNum++;
                                        break;
                                    case Block::BRAM_BLOCK :
                                        BRAMNum++;
                                        break;
                                    case Block::DSP_BLOCK :
                                        DSPNum++;
                                        break;
                                    case Block::FORBIDDEN_BLOCK :
                                        FORBBlock++;
                                        break;
                                }
                            }
                        }

                        if (FORBBlock > 0) continue; //If there is a forbidden block the solution is not feasible

                        if (CLBNum < problemRegion->getCLBNum() ||
                            BRAMNum < problemRegion->getBRAMNum() ||
                            DSPNum < problemRegion->getDSPNum()) {
                            continue;       //The area does not have the required blocks
                        }

                        if(regionType == RegionType::PR && (h % tileHeight != 0 || t % tileHeight != 0))
                            continue;

                        FeasiblePlacement *newPlacement = new FeasiblePlacement();
                        Point2D startPosition, dimension;

                        startPosition.set_x(j);
                        startPosition.set_y(t);
                        newPlacement->setStartPosition(startPosition);

                        dimension.set_x(k - j +1);
                        dimension.set_y(h);
                        newPlacement->setDimension(dimension);

                        newPlacement->setRegionType(problemRegion->getType());

                        newPlacement->setAreaCost(
                                static_cast<unsigned int>(CLBNum * problem->getCLBCost() +
                                                          BRAMNum * problem->getBRAMCost() +
                                                          DSPNum * problem->getDSPCost())
                        );


                        //If the aspect ratio is to high or to little the region is removed
                        /*float xdy = ((float)newPlacement->dimension.get_x() / (float)newPlacement->dimension.get_y());
                        float ydx = ((float)newPlacement->dimension.get_y() / (float)newPlacement->dimension.get_x());

                        if(xdy < 0.2 || ydx < 0.2)
                            continue;*/

                        /*Check if there are some areas that contains the just founded area
                        * or if there is at least one area that contain the just founded area
                        * In the first case the bigger areas must be removed since we have found a better solution
                        * In the second case the just founded solution is not a good solution since we already
                        * have found one area that match the requirements and is contained in the one that i have just found
                        **/
                        bool foundLittler = false;
                        for (std::list<FeasiblePlacement>::iterator it = feasiblePlacementsArray[i].begin();
                             it != feasiblePlacementsArray[i].end();
                             it++) {
                            //check if the new founded area is contained in another one
                            if(newPlacement->checkContains(&(*it))){
                                foundLittler = true;
                                break;
                            }
                        }

                        //If found a littler one the new founded area is useless
                        if(foundLittler) break;


                        std::list<FeasiblePlacement> elementsToRemove;
                        for (std::list<FeasiblePlacement>::iterator it = feasiblePlacementsArray[i].begin();
                             it != feasiblePlacementsArray[i].end();
                             it++) {
                            //check if the new founded area is contained in another one
                            if(it->checkContains(newPlacement)){
                                elementsToRemove.push_back(*it);
                            }
                        }


                        //Remove all the elements to remove
                        for (std::list<FeasiblePlacement>::iterator it = elementsToRemove.begin(); it != elementsToRemove.end() ; ++it) {
                            feasiblePlacementsArray[i].erase(it);
                        }
                        //feasiblePlacementsArray[i].erase(elementsToRemove.begin(), elementsToRemove.end());

                        //add the new found area
                        feasiblePlacementsArray[i].push_back(*newPlacement);
                    }
                }
            }
        }
    }


    //Resize the vector of all the feasible placements
    // to make it able to fit all the regions
    /*feasiblePlacements->resize(problem->getNumRegions());
    for (int j1 = 0; j1 < problem->getNumRegions(); ++j1) {
        feasiblePlacements->at(j1).resize(feasiblePlacementsArray[j1].size());
    }*/

    std::cout<<"Building final feasible regions array"<<std::endl;

    feasiblePlacements->resize(problem->getNumRegions());
    for (int i = 0; i < problem->getNumRegions(); ++i) {
        //For each region of the problem
        std::cout << i << std::endl;

        for (std::list<FeasiblePlacement>::iterator it = feasiblePlacementsArray[i].begin();
             it != feasiblePlacementsArray[i].end();
             it++) {
            feasiblePlacements->at(i).push_back(*it);
        }

    }

    std::cout<<". . . Finish"<< std::endl;

    //Deleting useless generated linked lists
    delete[] feasiblePlacementsArray;

}

int main() {
    cout << "*------* STARTING *------*" << endl;
    cout << "Loading data problem from file\n" << endl;
    Problem* problem;
    try {
        problem = FileManager::getINSTANCE().readProblem("/Users/Marco/CLionProjects/FloorplanningContestGeneticAlgorithm/Problems/10016");
    }catch ( const std::invalid_argument& e ){
        fprintf(stderr, e.what());
    }

    std::vector<std::vector<FeasiblePlacement>> feasiblePlacements;
    getAllFeasiblePlacements(&feasiblePlacements, problem);

    //FileManager::getINSTANCE().readFeasiblePlacementToFile("/Users/Marco/CLionProjects/BubbleRegionsFloorplanner/cmake-build-debug/10011Regions.txt",&feasiblePlacements);
    FileManager::getINSTANCE().writeFeasiblePlacementToFile(feasiblePlacements, problem);

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
            int numWire = std::max(problem->getInterconnectionsMatrix(i,j), problem->getInterconnectionsMatrix(j,i) );

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

    //Evaluate score impact multiplier for each region
    float totCost = 0;
    for (int k = 0; k < numRegions; ++k) {
        //For each region
        ProblemRegion* region1 = const_cast<ProblemRegion *>(problem->getFloorplanProblemRegion(k));

        float scoreValue = region1->getBRAMNum() * problem->getBRAMCost() +
                region1->getCLBNum() * problem->getCLBCost() +
                region1->getDSPNum() * problem->getDSPCost();
        scoreValue *= problem->getAreaCost();

        float wireScore = 0;
        for (int i = 0; i < region1->getIONum(); ++i) {
            wireScore += region1->getRegionIO()[i].getNumWires();
        }

        for (int j = 0; j < numRegions; ++j) {
            wireScore += problem->getInterconnectionsMatrix(k,j) + problem->getInterconnectionsMatrix(j,k);
        }

        wireScore *= problem->getWireCost();

        Point2D boardDim = problem->getBoard()->getDimension();
        float wireCostCoff = std::sqrt(boardDim.get_x() * boardDim.get_x() + boardDim.get_y() * boardDim.get_y());
        wireCostCoff *= 0.15;

        wireScore *= wireCostCoff;

        totCost += wireScore + scoreValue;

        regions[k].setScoreImpactMultiplier(wireScore + scoreValue);
    }

    for (int l = 0; l < numRegions; ++l) {
        regions[l].setScoreImpactMultiplier(regions[l].getScoreImpactMultiplier() / totCost * numRegions );
        regions[l].setScoreImpactMultiplier(std::exp((regions[l].getScoreImpactMultiplier()-1)));
    }

    Physics::getINSTANCE().setBoard(problem->getBoard());

    FloorplanningManager::getINSTANCE().setProblem(problem);
    FloorplanningManager::getINSTANCE().start();

    MainLoopManager::getINSTANCE().startLoop();

    return 0;
}
