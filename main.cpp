#include <iostream>
#include "Solving/PhysicsRegion.h"
#include "Data/ProblemData/Problem.h"
#include "FileManagement/FileManager.h"
#include "Solving/MainLoopManager.h"
#include "Solving/FloorplanningManager.h"
#include <list>
#include <cmath>

using namespace std;

void getAllFeasiblePlacements2(std::vector<std::vector<FeasiblePlacement>>* feasiblePlacements, Problem* problem){
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



                        //Check if there are some areas that contains the just founded area
                        // or if there is at least one area that contain the just founded area
                        // In the first case the bigger areas must be removed since we have found a better solution
                        // In the second case the just founded solution is not a good solution since we already
                        // have found one area that match the requirements and is contained in the one that i have just found
                        //
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


void getAllFeasiblePlacements(std::vector<std::vector<FeasiblePlacement>>* feasiblePlacements, Problem* problem){

    Board* board = problem->getBoard();
    char tileHeight = board->getTileHeight();

    int minRegionHeight[problem->getNumRegions()];
    bool alreadyFound[problem->getNumRegions()];


    //For each pair of column indexes
    for (int i = 0; i < board->getDimension().get_y(); ++i) {

        //Reset min region height values
        std::fill(minRegionHeight, minRegionHeight + problem->getNumRegions(), std::numeric_limits<int>::max());

        for (int j = i+1  ; j < board->getDimension().get_y(); ++j) {

            bool isPRColumn = problem->getLeftValidIDs(i) && problem->getRightValidIDs(j);

            //For each possible translation
            for (int t = 0; t <= board->getDimension().get_x(); ++t) {

                std::fill(alreadyFound, alreadyFound + problem->getNumRegions(), false);

                //For each possible height
                for (int h = 1; h <= board->getDimension().get_x() - t; ++h) {

                    //Get resource value
                    Resources res = board->getBoardResources( Point2D(t, i) , Point2D(t+h-1, j) );

                    if(res.FORBIDDEN > 0) //Found a forbidden.. no more possible expansions
                        break;

                    bool isTranslationHeightPRCompl = t % tileHeight == 0 && h % tileHeight == 0;

                    //For each region of the problem
                    for (int r = 0; r < problem->getNumRegions(); ++r) {
                        if(minRegionHeight[r] <= h)
                            continue;

                        if(alreadyFound[r])
                            continue;

                        ProblemRegion* problemRegion = const_cast<ProblemRegion *>(problem->getFloorplanProblemRegion(r));

                        if(problemRegion->getType() == RegionType::PR &&
                           (!isTranslationHeightPRCompl || !isPRColumn))
                            continue;

                        if(
                                problemRegion->getCLBNum() <= res.CLB &&
                                problemRegion->getDSPNum() <= res.DSP &&
                                problemRegion->getBRAMNum() <= res.BRAM
                                ){

                            alreadyFound[r] = true;
                            minRegionHeight[r] = std::min(minRegionHeight[r], h);

                            FeasiblePlacement newFp;

                            newFp.setDimension(Point2D(j - i + 1, h));

                            newFp.setRegionType(problemRegion->getType());

                            newFp.setAreaCost(
                                    static_cast<unsigned int>(res.CLB * problem->getCLBCost() +
                                                                        res.DSP * problem->getDSPCost() +
                                                                        res.BRAM * problem->getBRAMCost())
                            );

                            //Check for all the translations that do not contain a forbidden block
                            for (int t2 = t; t2 <= board->getDimension().get_x() - h; ++t2) {
                                if(problemRegion->getType() == RegionType::PR && t2 % tileHeight != 0)
                                    continue;

                                Resources res2 = board->getBoardResources( Point2D(t2, i) , Point2D(t2+h-1, j) );

                                if(res2.FORBIDDEN > 0)
                                    continue;

                                newFp.setStartPosition(Point2D(i, t2));
                                newFp.setResources(res2);

                                feasiblePlacements->at(r).push_back(newFp);
                            }
                        }
                    }

                }

            }
        }
    }
}

int main() {
    time_t seconds;
    seconds = time (NULL);

    cout << "*------* STARTING *------*" << endl;
    cout << "Loading data problem from file\n" << endl;
    Problem* problem;
    try {
        problem = FileManager::getINSTANCE().readProblem("/Users/Marco/CLionProjects/BubbleRegionsFloorplanner/RealProblems/25_85.txt");
    }catch ( const std::invalid_argument& e ){
        fprintf(stderr, e.what());
    }

    int boardBlockCounter = 0;
    int problemBlockCounter = 0;

    Board* board = problem->getBoard();
    board->computeCumulativeResourceMatrix();

    for (int i = 0; i < board->getDimension().get_x(); ++i) {
        for (int j = 0; j < board->getDimension().get_y(); ++j) {
            Block block = board->getBlockMatrix(i,j);
            if(block == Block::CLB_BLOCK || block == Block::DSP_BLOCK || block == Block::BRAM_BLOCK)
                boardBlockCounter++;
        }
    }

    for (int i = 0; i < problem->getNumRegions(); ++i) {
        ProblemRegion* pr = const_cast<ProblemRegion *>(problem->getFloorplanProblemRegion(i));
        problemBlockCounter += pr->getCLBNum() + pr->getDSPNum() + pr->getBRAMNum();
    }

    float percentage = ((float)problemBlockCounter / (float)boardBlockCounter);
    std::cout << "Density: " << percentage * 100 <<"%"<<std::endl;

    std::vector<std::vector<FeasiblePlacement>> feasiblePlacements;
    feasiblePlacements.resize(problem->getNumRegions());
    getAllFeasiblePlacements(&feasiblePlacements, problem);

    //FileManager::getINSTANCE().readFeasiblePlacementToFile("/Users/Marco/CLionProjects/BubbleRegionsFloorplanner/cmake-build-debug/10020Regions.txt", &feasiblePlacements);
    FileManager::getINSTANCE().writeFeasiblePlacementToFile(feasiblePlacements, problem);

    time_t seconds2;
    seconds2 = time (NULL);
    std::cout<<"Feasible placements search time : "<<seconds2 - seconds<<std::endl;
    seconds = seconds2;

    //Keep only best regions by area
    /*
    for (int i = 0; i < feasiblePlacements.size(); ++i) {
        std::list<FeasiblePlacement> placementsList{std::make_move_iterator(
                std::begin(feasiblePlacements.at(i))), std::make_move_iterator(std::end(feasiblePlacements.at(i)))
        };

        int bestAreaValue = std::numeric_limits<unsigned short>::max();

        for (std::list<FeasiblePlacement>::iterator it = placementsList.begin(); it != placementsList.end(); ++it) {
            FeasiblePlacement fp = *it;

            unsigned short area = fp.getDimension().get_x() * fp.getDimension().get_y();

            if (area < bestAreaValue) {
                bestAreaValue = area;
            }
        }

        placementsList.remove_if([bestAreaValue](FeasiblePlacement placement) {
            unsigned short area = placement.getDimension().get_x() * placement.getDimension().get_y();
            return area > bestAreaValue * 1.2;
        });

        std::vector<FeasiblePlacement> newVector{placementsList.begin(), placementsList.end()};
        feasiblePlacements.at(i) = newVector;
    }*/


    //float maxArea = 2*std::exp(-problem->getNumRegions()/20)+1.05;
    float maxArea = 1.0 / 2.0 / percentage + 1.0 / 2.0;
    std::cout<<"Problem max area: "<<maxArea<<std::endl;
    for (int i = 0; i < feasiblePlacements.size(); ++i) {
        std::vector<FeasiblePlacement> placementVector = feasiblePlacements.at(i);

        //Search for best area value
        int bestAreaValue = std::numeric_limits<unsigned short>::max();
        for (int j = 0; j < placementVector.size(); ++j) {
            FeasiblePlacement fp = placementVector.at(j);

            unsigned short area = fp.getDimension().get_x() * fp.getDimension().get_y();

            if(area < bestAreaValue){
                bestAreaValue = area;
            }
        }

        //Eliminate placements with area bigger than bestAreaValue + x%
        int l = 0;
        for (int k = 0; k < feasiblePlacements.at(i).size(); ++k) {
            FeasiblePlacement fp = feasiblePlacements.at(i).at(k);


            unsigned short area = fp.getDimension().get_x() * fp.getDimension().get_y();

            if(area <= bestAreaValue * 1.1) {
                feasiblePlacements.at(i).at(l) = placementVector.at(k);
                l++;
            }
        }
        feasiblePlacements.at(i).resize(l);
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


    seconds2 = time (NULL);
    std::cout<<"Regions elimination time : "<<seconds2 - seconds<<std::endl;

    Physics::getINSTANCE().setBoard(problem->getBoard());


    FloorplanningManager::getINSTANCE().setProblem(problem);
    FloorplanningManager::getINSTANCE().start();

    MainLoopManager::getINSTANCE().startLoop();

    return 0;
}
