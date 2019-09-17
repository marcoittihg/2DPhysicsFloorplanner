#include <iostream>
#include "Solving/PhysicsRegion.h"
#include "Data/ProblemData/Problem.h"
#include "FileManagement/FileManager.h"
#include "Solving/MainLoopManager.h"
#include "Solving/FloorplanningManager.h"
#include <list>
#include <cmath>


using namespace std;


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

    FloorplanningManager::getINSTANCE().setStartTime(seconds);

    cout << "*------* STARTING *------*" << endl;
    cout << "Loading data problem from file\n" << endl;
    Problem* problem;

    try {
        problem = FileManager::getINSTANCE().readProblem("/Users/Marco/CLionProjects/BubbleRegionsFloorplanner/Problems/10020");
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
    //FileManager::getINSTANCE().writeFeasiblePlacementToFile(feasiblePlacements, problem);

    time_t seconds2;
    seconds2 = time (NULL);
    std::cout<<"Feasible placements search time : "<<seconds2 - seconds<<std::endl;
    seconds = seconds2;

    //Keep only best regions by area
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

            float area = fp.getDimension().get_x() * fp.getDimension().get_y();
            //if(area <= bestAreaValue * 9999) {
            if((fp.getDimension().get_y() <= 9999 && fp.getStartPosition().get_x() != 0) || area <= bestAreaValue * 1.05){
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
