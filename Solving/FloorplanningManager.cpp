//
// Created by Marco on 31/12/17.
//

#include <thread>
#include <iostream>
#include <cmath>
#include <set>
#include <random>
#include "FloorplanningManager.h"
#include "PhysicsRegion.h"

void FloorplanningManager::start() {
    _time = 0;
    wireStabTime = 110.0;
    wireStabImprovementTime = 1.0;
    closestAlternativeRefreshTime = 60.0;
    lastAlternativeRefreshTime = 0.0;

    realWireStabPositionsImprovementTime = 0;

    placemStabTime = 60.0f;

    state = FloortplanningMangerState::START;

    oldWireStabRegionPos = new Vector2[Physics::getINSTANCE().getRegionNum()];
    oldWireStabWirelen = std::numeric_limits<float>::max();

    solutions = new FloorplanSolution[numSolutions];
    for (int i = 0; i < numSolutions; ++i) {
        solutions[i].score = std::numeric_limits<int>::min();
        solutions[i].placementsIndexes = new int[Physics::getINSTANCE().getRegionNum()];
    }

    bestSolutionScore = std::numeric_limits<int>::min();

    minDisplacePercentage = 0.1;
    maxDisplacePercentage = 0.9;
}

void FloorplanningManager::onPysicsStep() {
    _time += Physics::getINSTANCE().getFIXED_STEP_TIME();

    if(state == FloortplanningMangerState::START){
        //Check how many floating regions there are
        int floatingReg = 0;

        PhysicsRegion* regions = Physics::getINSTANCE().getPhysicsRegions();
        int regionNum = problem->getNumRegions();

        for (int i = 0; i < regionNum; ++i) {
            if(regions[i].getRegionState() == PhysicsRegionState::FLOATING) {
                regions[i].resetPositionAndShape();
                unsigned short blockNum = static_cast<unsigned short>(
                        problem->getFloorplanProblemRegion(i)->getDSPNum() +
                        problem->getFloorplanProblemRegion(i)->getCLBNum() +
                        problem->getFloorplanProblemRegion(i)->getCLBNum());

                float radius = std::sqrt(blockNum / M_PI);

                regions[i].getRb()->setDimension(Vector2(radius,radius));
                floatingReg++;
            }
        }

        //Set coefficients
        Physics::getINSTANCE().setPreferedAnchorCoeff(0);
        Physics::getINSTANCE().setClosestAnchorCoeff(0);
        Physics::getINSTANCE().setSeparationCoeff(0);
        Physics::getINSTANCE().setWireForceCoeff(1);
        Physics::getINSTANCE().setNoiseModulus(0);
        Physics::getINSTANCE().setNoiseSpeedCoeff(0);

        if(false){/*problem->getWireCost() == 0*/
            for (int i = 0; i < regionNum; ++i) {
                if (regions[i].getRegionState() == PhysicsRegionState::FLOATING) {
                    //If the region is floating evaluate the new placement and shape
                    regions[i].evaluatePlacementAndShape(true);
                }
            }

            //Change state
            state = FloortplanningMangerState::SEARCH_PLACEM;
            _time = 0;
            Physics::getINSTANCE().setNoiseSpeedCoeff(10);
            Physics::getINSTANCE().setNoiseModulus(0);
            Physics::getINSTANCE().setWireForceCoeff(0);
        }else{
            //Change state for next iteration
            state = FloortplanningMangerState::WAITING_FOR_WIRE_STABILITY;
            Physics::getINSTANCE().setWireForceCoeff(1);
            Physics::getINSTANCE().setEnableBarrierCollisions(true);
            _time = 0;
            Physics::getINSTANCE().setIoForceMultiplier(1);
            tmpTime = time(NULL);

            phaseStartTime = time(NULL);
        }
    } else if(state == FloortplanningMangerState::WAITING_FOR_WIRE_STABILITY){
        std::cout<<_time<<std::endl;

        if(_time > 100)
            Physics::getINSTANCE().setSeparationCoeff(2000 * (_time - 100));
        else
            Physics::getINSTANCE().setSeparationCoeff(0);

        if(_time > wireStabTime){
            //Go to search placement state
            _time = 0;
            state = FloortplanningMangerState::WIRE_STABILITY_IMPROVEMENT;

            for (int i = 0; i < Physics::getINSTANCE().getRegionNum(); ++i) {
                oldWireStabRegionPos[i] = Physics::getINSTANCE().getPhysicsRegions()[i].getRb()->getPosition();
            }
        }

    }else if(state == FloortplanningMangerState::WIRE_STABILITY_IMPROVEMENT){
        std::cout<<_time<<std::endl;

        Physics::getINSTANCE().setSeparationCoeff(2000 * (_time + wireStabTime - wireStabImprovementTime - 100));

        //if the time is up.. go to the search phase
        if(_time > wireStabImprovementTime){

            cont++;
            std::cout<<"Attempt "<<cont<<std::endl;
            time_t now = time(NULL);
            PhysicsRegion* regions = Physics::getINSTANCE().getPhysicsRegions();
            int regionNum = Physics::getINSTANCE().getRegionNum();
            std::cout<<"a";
            if(now-phaseStartTime >= realWireStabPositionsImprovementTime){
                std::cout<<"b";
                //Go to search placements stage
                _time = 0;
                state = FloortplanningMangerState::SEARCH_PLACEM;

                Physics::getINSTANCE().setEnableRegionCollisions(true);
                Physics::getINSTANCE().setWireForceCoeff(0);
                Physics::getINSTANCE().setEnableBarrierCollisions(false);

                time_t wireStabEndTime = time(NULL);
                std::cout<<"Wire stability search time : "<<wireStabEndTime - tmpTime<<std::endl;
                tmpTime = wireStabEndTime;


                //Save wire stability position for each floating region
                for (int i = 0; i < regionNum; ++i) {
                    regions[i].getRb()->setPosition(oldWireStabRegionPos[i]);
                }
                for (int i = 0; i < regionNum; ++i) {
                    if(regions[i].getRegionState() == PhysicsRegionState::FLOATING) {
                        regions[i].savePositionAsWireStability();
                    }
                }

                //Evaluate each region
                for (int i = 0; i < regionNum; ++i) {
                    if(regions[i].getRegionState() == PhysicsRegionState::FLOATING) {
                        //If the region is floating evaluate the new placement and shape
                        regions[i].evaluatePlacementAndShape(true);
                        regions[i].getRb()->setPosition(regions[i].getPreferedAnchorPoint());
                        regions[i].getRb()->setSpeed(Vector2(0,0));
                    }
                }
                return;
            }

            Point2D boardDim = Physics::getINSTANCE().getBoard()->getDimension();
            Vector2 minusHalfBoardDim = Vector2(-(float)boardDim.get_y() / 2, -(float)boardDim.get_x() / 2);

            //Calculate actual wire stability wirelen
            float wirelen = 0;

            for (int i = 0; i < regionNum; ++i) {
                Vector2 regionPos = regions[i].getRb()->getPosition();

                std::vector<PhysicsRegion*> intercRegions = regions[i].getInterconnectedRegions();
                for (int j = 0; j < intercRegions.size(); ++j) {
                    Vector2 dist = regionPos;
                    dist.multiply(-1);
                    dist.add(intercRegions.at(j)->getRb()->getPosition());

                    float halfDistance = dist.mangnitude() * 0.5;

                    wirelen += halfDistance * regions[i].getInterconnectedRegionsWeights().at(j);
                }

                RegionIOData* regionIODatas = regions[i].getRegionIO();


                for (int k = 0; k < regions[i].getIONum(); ++k) {
                    Vector2 IOPos = Vector2(regionIODatas[k].getPortColumn(), regionIODatas[k].getPortRow());
                    IOPos.add(minusHalfBoardDim);
                    IOPos.multiply(-1);
                    IOPos.add(regionPos);

                    float IODistance = IOPos.mangnitude();

                    wirelen += IODistance * regionIODatas[k].getNumWires();
                }
            }

            //Save positions as new best if there is an improvement
            if(wirelen < oldWireStabWirelen){
                std::cout<<"Improved!..save"<< wirelen <<std::endl;
                oldWireStabWirelen = wirelen;

                for (int i = 0; i < regionNum; ++i) {
                    oldWireStabRegionPos[i] = regions[i].getRb()->getPosition();
                }
            }else{
                //Otherwise restore old positions
                for (int i = 0; i < regionNum; ++i) {
                    regions[i].getRb()->setPosition(oldWireStabRegionPos[i]);
                }
            }

            //Set at 0 all the velocities
            for (int i = 0; i < regionNum; ++i) {
                regions[i].getRb()->setSpeed(Vector2(0,0));
            }

            //chose the number of moves to apply
            std::random_device r;
            std::mt19937_64 generator(r());
            std::uniform_int_distribution<int> movesDistribution(1, 3);
            int numMoves = movesDistribution(generator);


            //Chose the next improvement to try
            std::uniform_real_distribution<float> distribution(0,1);

            for (int m = 0; m < numMoves; ++m) {
                float value = distribution(generator);

                if (value < 0.5) {
                    //Move one regions at the center of its interconnections

                    //Select a random region
                    std::uniform_int_distribution<int> regionDistribution(0, regionNum - 1);
                    int regionIndex = regionDistribution(generator);

                    //Calculate weighted center
                    int numConnections =
                            regions[regionIndex].getInterconnectedRegions().size() + regions[regionIndex].getIONum();
                    Vector2 positions[numConnections];
                    int weights[numConnections];

                    RegionIOData *regionIODatas = regions[regionIndex].getRegionIO();
                    for (int i = 0; i < regions[regionIndex].getIONum(); ++i) {
                        Vector2 IOPos = Vector2(regionIODatas[i].getPortColumn(), regionIODatas[i].getPortRow());
                        IOPos.add(minusHalfBoardDim);

                        positions[i] = IOPos;
                        weights[i] = regionIODatas[i].getNumWires();
                    }

                    std::vector<PhysicsRegion *> intercRegions = regions[regionIndex].getInterconnectedRegions();
                    std::vector<int> intercRegionsWeights = regions[regionIndex].getInterconnectedRegionsWeights();
                    for (int j = 0; j < intercRegions.size(); ++j) {
                        positions[regions[regionIndex].getIONum() - 1 + j] = intercRegions[j]->getRb()->getPosition();
                        weights[regions[regionIndex].getIONum() - 1 + j] = intercRegionsWeights[j];
                    }

                    int weightSum = 0;
                    for (int k = 0; k < numConnections; ++k) {
                        positions[k].multiply(weights[k]);
                        weightSum += weights[k];
                    }

                    Vector2 meanVector = Vector2(0, 0);
                    for (int l = 0; l < numConnections; ++l) {
                        meanVector.add(positions[l]);
                    }

                    meanVector.multiply(((float) 1 / (float) weightSum));
                    regions[regionIndex].getRb()->setPosition(meanVector);

                } else {
                    //Try to swap two regions

                    //select the first region
                    std::uniform_int_distribution<int> regionDistribution(0, regionNum - 1);
                    int regionIndex1 = regionDistribution(generator);
                    int regionIndex2;

                    //Select the second region
                    do {
                        regionIndex2 = regionDistribution(generator);
                    } while (regionIndex1 == regionIndex2);

                    //Swap positions
                    Vector2 tmpPos = regions[regionIndex1].getRb()->getPosition();
                    regions[regionIndex1].getRb()->setPosition(regions[regionIndex2].getRb()->getPosition());
                    regions[regionIndex2].getRb()->setPosition(tmpPos);
                }
            }

            _time = 0;
        }
    }else if(state == FloortplanningMangerState::SEARCH_PLACEM){
        if(_time > 0)
            Physics::getINSTANCE().setSeparationCoeff((_time));
        else{
            Physics::getINSTANCE().setSeparationCoeff(0);
        }
        Physics::getINSTANCE().setPreferedAnchorCoeff(1);
        //Physics::getINSTANCE().setClosestAnchorCoeff(sqrt(time));
        //Physics::getINSTANCE().setNoiseModulus(sqrt(time));


        //If is passed enough time select the region that need to reevaluate his prefered choice
        PhysicsRegion* regions = Physics::getINSTANCE().getPhysicsRegions();
        int regionNum = problem->getNumRegions();
        if(_time - lastAlternativeRefreshTime > closestAlternativeRefreshTime) {

            //True if the region of index i has been already considered
            bool consideredRegions[regionNum];
            for (int i = 0; i < regionNum; ++i) {
                consideredRegions[i] = false;
            }

            for (int i = 0; i < regionNum; ++i) {
                if(!consideredRegions[i]){
                    if(regions[i].getRegionState() == PhysicsRegionState::PLACED) {
                        consideredRegions[i] = true;
                        continue;
                    }

                    //Check the collision group
                    std::set<int> collisionIndexes;
                    collisionIndexes.clear();
                    collisionIndexes.insert(i);
                    bool foundNew = true;
                    while (foundNew){
                        foundNew = false;
                        for (int j = 0; j < regionNum; ++j){
                            if(collisionIndexes.find(j) != collisionIndexes.end())
                                continue;

                            FeasiblePlacement regionPlacement = regions[j].getFeasiblePlacements()[regions[j].getPreferdPlacementIndex()];

                            for (std::set<int>::iterator it = collisionIndexes.begin(); it != collisionIndexes.end(); ++it) {
                                if(regions[*it].getRegionState() == PhysicsRegionState ::PLACED)
                                    continue;

                                FeasiblePlacement collIndPlacement = regions[*it].getFeasiblePlacements()[regions[*it].getPreferdPlacementIndex()];

                                bool collision = FeasiblePlacement::checkCollision(&regionPlacement, &collIndPlacement, Physics::getINSTANCE().getBoard()->getTileHeight());

                                if(collision){
                                    foundNew = true;
                                    collisionIndexes.insert(j);
                                    break;
                                }
                            }
                        }
                    }

                    //find the closest placement of the just found collision group
                    float minDistance = std::numeric_limits<float>::max();
                    int regionIndex = -1;
                    for (std::set<int>::iterator it = collisionIndexes.begin(); it != collisionIndexes.end(); ++it) {
                        Vector2 minusAchorPos = regions[*it].getPreferedAnchorPoint();
                        minusAchorPos.multiply(-1);

                        minusAchorPos.add(regions[*it].getRb()->getPosition());

                        float distance = minusAchorPos.mangnitude();

                        if(minDistance > distance){
                            minDistance = distance;
                            regionIndex = *it;
                        }
                    }

                    //Place the region closest to its anchor point
                    regions[regionIndex].setRegionState(PhysicsRegionState::PLACED);
                    regions[regionIndex].getRb()->setPosition(regions[regionIndex].getPreferedAnchorPoint());


                    //All the placed regions that overlap with the just chosen region are no longer placed
                    FeasiblePlacement newRegionPlacement = regions[regionIndex].getFeasiblePlacements()[regions[regionIndex].getPreferdPlacementIndex()];

                    for (int k = 0; k < regionNum; ++k) {
                        if(k == regionIndex)
                            continue;
                        FeasiblePlacement regionPlacement = regions[k].getFeasiblePlacements()[regions[k].getPreferdPlacementIndex()];

                        if(FeasiblePlacement::checkCollision(&newRegionPlacement, &regionPlacement, Physics::getINSTANCE().getBoard()->getTileHeight())){
                            regions[k].setRegionState(PhysicsRegionState::FLOATING);
                        }
                    }

                    //Update the considered regions
                    for (std::set<int>::iterator it = collisionIndexes.begin(); it != collisionIndexes.end(); ++it) {
                        consideredRegions[*it] = true;
                    }
                }
            }

            //For every placed region check if is available a better and free placement
            bool improved = true;
            while(improved) {
                improved = false;
                for (int m = 0; m < regionNum; ++m) {
                    if (regions[m].getRegionState() != PhysicsRegionState::PLACED)
                        continue;

                    FeasiblePlacement nowPlacement = regions[m].getFeasiblePlacements()[regions[m].getPreferdPlacementIndex()];
                    unsigned int nowScore = regions[m].evaluatePlacement(nowPlacement, false);
                    unsigned int bestScore = nowScore;
                    int index = -1;

                    for (int i = 0; i < regions[m].getPlacementNum(); ++i) {
                        FeasiblePlacement placement = regions[m].getFeasiblePlacements()[i];

                        //Check if the placement overlap with the taken placement of an other placed regions
                        bool found = false;
                        for (int j = 0; j < regionNum; ++j) {
                            if (j == m || regions[j].getRegionState() != PhysicsRegionState::PLACED)
                                continue;

                            FeasiblePlacement otherFP = regions[j].getFeasiblePlacements()[regions[j].getPreferdPlacementIndex()];
                            if (FeasiblePlacement::checkCollision(&placement, &otherFP,
                                                                  Physics::getINSTANCE().getBoard()->getTileHeight())) {
                                found = true;
                                break;
                            }
                        }

                        if (found)
                            continue;

                        unsigned int score = regions[m].evaluatePlacement(placement, false);

                        if (score < bestScore) {
                            bestScore = score;
                            index = i;
                        }
                    }

                    if (bestScore < nowScore) {
                        //Found a better placement
                        FeasiblePlacement newPlacement = regions[m].getFeasiblePlacements()[index];

                        Point2D boardDim = Physics::getINSTANCE().getBoard()->getDimension();
                        Vector2 minusHalfBoardDim = Vector2(-(float) boardDim.get_y() / 2,
                                                            -(float) boardDim.get_x() / 2);

                        Vector2 regionPos = Vector2(newPlacement.getStartPosition().get_x(),
                                                    newPlacement.getStartPosition().get_y());
                        regionPos.add(minusHalfBoardDim);

                        Vector2 halfDim = Vector2((float) newPlacement.getDimension().get_x() * 0.5,
                                                  (float) newPlacement.getDimension().get_y() * 0.5);
                        regionPos.add(halfDim);

                        regions[m].getRb()->setPosition(regionPos);
                        regions[m].getRb()->setDimension(
                                Vector2(newPlacement.getDimension().get_x(), newPlacement.getDimension().get_y()));

                        regions[m].setPreferdPlacementIndex(index);
                        regions[m].setPreferedAnchorPoint(regionPos);

                        improved = true;
                    }


                }
            }

            //For every non placed region evaluate its prefered placement
            for (int k = 0; k < regionNum; ++k) {
                if(regions[k].getRegionState() == PhysicsRegionState::PLACED)
                    continue;

                regions[k].evaluatePlacementAndShape(false);
                regions[k].getRb()->setPosition(regions[k].getPreferedAnchorPoint());
                regions[k].getRb()->setSpeed(Vector2(0,0));
            }

            //Check if there is at least one floating region
            bool found = false;
            for (int l = 0; l < regionNum; ++l) {
                if(regions[l].getRegionState() == PhysicsRegionState::FLOATING) {
                    found = true;
                    break;
                }
            }

            if(!found){
                state = FloortplanningMangerState ::END;
            }
            _time = 0;
        }
    }else if(state == FloortplanningMangerState::END){
        PhysicsRegion* regions = Physics::getINSTANCE().getPhysicsRegions();
        int regionNum = problem->getNumRegions();

        //Improve the just found solution
        bool improved = true;
        while(improved) {
            improved = false;
            for (int m = 0; m < regionNum; ++m) {
                if (regions[m].getRegionState() != PhysicsRegionState::PLACED)
                    continue;

                FeasiblePlacement nowPlacement = regions[m].getFeasiblePlacements()[regions[m].getPreferdPlacementIndex()];
                unsigned int nowScore = regions[m].evaluatePlacement(nowPlacement, true);
                unsigned int bestScore = nowScore;
                int index = -1;

                for (int i = 0; i < regions[m].getPlacementNum(); ++i) {
                    FeasiblePlacement placement = regions[m].getFeasiblePlacements()[i];

                    //Check if the placement overlap with the taken placement of an other placed regions
                    bool found = false;
                    for (int j = 0; j < regionNum; ++j) {
                        if (j == m || regions[j].getRegionState() != PhysicsRegionState::PLACED)
                            continue;

                        FeasiblePlacement otherFP = regions[j].getFeasiblePlacements()[regions[j].getPreferdPlacementIndex()];
                        if (FeasiblePlacement::checkCollision(&placement, &otherFP,
                                                              Physics::getINSTANCE().getBoard()->getTileHeight())) {
                            found = true;
                            break;
                        }
                    }

                    if (found)
                        continue;

                    unsigned int score = regions[m].evaluatePlacement(placement, true);

                    if (score < bestScore) {
                        bestScore = score;
                        index = i;
                    }
                }

                if (bestScore < nowScore) {
                    //Found a better placement
                    FeasiblePlacement newPlacement = regions[m].getFeasiblePlacements()[index];

                    Point2D boardDim = Physics::getINSTANCE().getBoard()->getDimension();
                    Vector2 minusHalfBoardDim = Vector2(-(float) boardDim.get_y() / 2,
                                                        -(float) boardDim.get_x() / 2);

                    Vector2 regionPos = Vector2(newPlacement.getStartPosition().get_x(),
                                                newPlacement.getStartPosition().get_y());
                    regionPos.add(minusHalfBoardDim);

                    Vector2 halfDim = Vector2((float) newPlacement.getDimension().get_x() * 0.5,
                                              (float) newPlacement.getDimension().get_y() * 0.5);
                    regionPos.add(halfDim);

                    regions[m].getRb()->setPosition(regionPos);
                    regions[m].getRb()->setDimension(
                            Vector2(newPlacement.getDimension().get_x(), newPlacement.getDimension().get_y()));

                    regions[m].setPreferdPlacementIndex(index);
                    regions[m].setPreferedAnchorPoint(regionPos);

                    improved = true;
                }


            }
        }

        //Evaluate the just found solution
        int score = problem->getMaxScore();

        Point2D boardDim = Physics::getINSTANCE().getBoard()->getDimension();
        Vector2 minusHalfBoardDim = Vector2(-(float)boardDim.get_y() / 2, -(float)boardDim.get_x() / 2);

        for (int k = 0; k < regionNum; ++k) {
            Vector2 regionPos = regions[k].getRb()->getPosition();

            score -= regions[k].getFeasiblePlacements()[regions[k].getPreferdPlacementIndex()].getAreaCost() * problem->getAreaCost();

            RegionIOData* ioData = regions[k].getRegionIO();
            for (int i = 0; i < regions[k].getIONum(); ++i) {
                Vector2 IOPos = Vector2(ioData[i].getPortColumn(), ioData[i].getPortRow());
                IOPos.add(minusHalfBoardDim);
                IOPos.multiply(-1);
                IOPos.add(regionPos);

                float distX = std::abs(IOPos.getX());
                float distY = std::abs(IOPos.getY());

                score -= (distX + distY) * ioData[i].getNumWires() * problem->getWireCost();
            }

            std::vector<PhysicsRegion*> intercRegions = regions[k].getInterconnectedRegions();
            std::vector<int> intercRegionsWeights = regions[k].getInterconnectedRegionsWeights();
            for (int j = 0; j < intercRegions.size(); ++j) {
                PhysicsRegion* intercRegion = intercRegions.at(j);

                Vector2 dist = regionPos;
                dist.multiply(-1);
                dist.add(intercRegions.at(j)->getRb()->getPosition());

                float distX = std::abs(dist.getX());
                float distY = std::abs(dist.getY());

                float halfDistance = (distX + distY) * 0.5;

                score -= halfDistance * intercRegionsWeights.at(j) * problem->getWireCost();
            }
        }

        //Save the solution if better than an already existing one
        int index = -1;
        for (int l = 0; l < numSolutions; ++l) {
            if(solutions[l].score < score){
                index = l;
                break;
            }
        }

        if(index != -1){
            //Check if the same solution already exist
            bool found = false;
            for (int j = 0; j < numSolutions; ++j) {
                bool differentPlacementFound = false;

                for (int i = 0; i < regionNum; ++i) {
                    if(solutions[index].placementsIndexes[i] != regions[i].getPreferdPlacementIndex()){
                        differentPlacementFound = true;
                        break;
                    }
                }
                if(!differentPlacementFound) {
                    found = true;
                    break;
                }
            }

            if(!found) {
                //Save the solution
                solutions[index].score = score;

                for (int i = 0; i < regionNum; ++i) {
                    solutions[index].placementsIndexes[i] = regions[i].getPreferdPlacementIndex();
                }
            }
        }

        //Print the best solution if there is an improvement
        int best = std::numeric_limits<int>::min();
        int bestIndex = -1;
        for (int l = 0; l < numSolutions; ++l) {
            if(solutions[l].score > best){
                best = solutions[l].score;
                bestIndex = l;
            }
        }

        if(best > bestSolutionScore){
            bestSolutionScore = best;

            //Print the new best solutions
            std::cout<<"Found new best solution"<<std::endl;


            time_t placementSearchEndTime = time(NULL);
            std::cout<<"Elapse : "<<placementSearchEndTime - startTime<<std::endl;


            std::cout<<"Score: "<<bestSolutionScore<<std::endl;

            for (int i = 0; i < regionNum; ++i) {
                FeasiblePlacement fp = regions[i].getFeasiblePlacements()[regions[i].getPreferdPlacementIndex()];

                Point2D start = fp.getStartPosition();
                Point2D dim = fp.getDimension();

                std::cout << +start.get_x() + 1 << " " << +start.get_y() + 1 << " "
                          << +dim.get_x() << " " << +dim.get_y() << std::endl;
            }
            std::cout << std::endl;
        }


        //Select the new solution to improve
        int improveIndex = -1;
        for (int n = 0; n < numSolutions; ++n) {
            if(solutions[n].score == std::numeric_limits<int>::min()){
                improveIndex = n;
                break;
            }
        }

        std::random_device r;
        std::mt19937_64 generator(r());
        std::uniform_real_distribution<float> uniform01(0, 1);

        float random = uniform01(generator);
        if(improveIndex == -1 && random > 0.05){
            std::uniform_int_distribution<int> uniform_dist(0, numSolutions-1);
            improveIndex = uniform_dist(generator);

            //Load the old solution
            FloorplanSolution solution = solutions[improveIndex];

            for (int i = 0; i < regionNum; ++i) {
                regions[i].setPreferdPlacementIndex(solution.placementsIndexes[i]);

                FeasiblePlacement fp = regions[i].getFeasiblePlacements()[solution.placementsIndexes[i]];

                regions[i].getRb()->setDimension(Vector2(fp.getDimension().get_x(),fp.getDimension().get_y()));

                Vector2 anchorPoint = Vector2(
                        fp.getStartPosition().get_x() + fp.getDimension().get_x() * 0.5,
                        fp.getStartPosition().get_y() + fp.getDimension().get_y() * 0.5
                );
                anchorPoint.add(minusHalfBoardDim);

                regions[i].setPreferedAnchorPoint(anchorPoint);
            }

            //Displace some regions
            std::uniform_real_distribution<float> displacePercentageDist(minDisplacePercentage, maxDisplacePercentage);

            int regionsToDisplace = (int)( std::floorf(displacePercentageDist(generator)* regionNum) + 1 );

            float scoreLossSum = 0;
            float regionsScoreLoss[regionNum];

            for (int j = 0; j < regionNum; ++j) {
                float scoreLoss = 0;

                Vector2 regionPos = regions[j].getRb()->getPosition();

                scoreLoss += regions[j].getFeasiblePlacements()[regions[j].getPreferdPlacementIndex()].getAreaCost() * problem->getAreaCost();

                RegionIOData* ioData = regions[j].getRegionIO();
                for (int i = 0; i < regions[j].getIONum(); ++i) {
                    Vector2 IOPos = Vector2(ioData[i].getPortColumn(), ioData[i].getPortRow());
                    IOPos.add(minusHalfBoardDim);
                    IOPos.multiply(-1);
                    IOPos.add(regionPos);

                    float distX = std::abs(IOPos.getX());
                    float distY = std::abs(IOPos.getY());

                    scoreLoss += (distX + distY) * ioData[i].getNumWires() * problem->getWireCost();
                }

                std::vector<PhysicsRegion*> intercRegions = regions[j].getInterconnectedRegions();
                std::vector<int> intercRegionsWeights = regions[j].getInterconnectedRegionsWeights();
                for (int j = 0; j < intercRegions.size(); ++j) {
                    Vector2 dist = regionPos;
                    dist.multiply(-1);
                    dist.add(intercRegions.at(j)->getRb()->getPosition());

                    float distX = std::abs(dist.getX());
                    float distY = std::abs(dist.getY());

                    float halfDistance =(distX+distY) * 0.5;

                    scoreLoss += halfDistance * intercRegionsWeights.at(j) * problem->getWireCost();
                }

                scoreLossSum += scoreLoss;
                regionsScoreLoss[j] = scoreLoss;
            }

            for (int k = 0; k < regionsToDisplace; ++k) {
                std::uniform_real_distribution<float> displaceDistr(0, scoreLossSum);
                float generatedLoss = displaceDistr(generator);

                float lossSum = regionsScoreLoss[0];
                int regionIndex = -1;
                for (int i = 0; i < regionNum && lossSum < scoreLossSum; ++i) {
                    lossSum += regionsScoreLoss[i];

                    if(lossSum > generatedLoss){
                        regionIndex = i;
                        break;
                    }
                }

                if(regionIndex == -1)
                    continue;

                if(regions[regionIndex].getRegionState() == PhysicsRegionState::FLOATING) //Already floating.. find another one
                    continue;

                regions[regionIndex].setRegionState(PhysicsRegionState::FLOATING);
            }

            //Initial local improvement
            bool improved = true;
            while(improved) {
                improved = false;
                for (int m = 0; m < regionNum; ++m) {
                    if (regions[m].getRegionState() != PhysicsRegionState::PLACED)
                        continue;

                    FeasiblePlacement nowPlacement = regions[m].getFeasiblePlacements()[regions[m].getPreferdPlacementIndex()];
                    unsigned int nowScore = regions[m].evaluatePlacement(nowPlacement, false);
                    unsigned int bestScore = nowScore;
                    int index = -1;

                    for (int i = 0; i < regions[m].getPlacementNum(); ++i) {
                        FeasiblePlacement placement = regions[m].getFeasiblePlacements()[i];

                        //Check if the placement overlap with the taken placement of an other placed regions
                        bool found = false;
                        for (int j = 0; j < regionNum; ++j) {
                            if (j == m || regions[j].getRegionState() != PhysicsRegionState::PLACED)
                                continue;

                            FeasiblePlacement otherFP = regions[j].getFeasiblePlacements()[regions[j].getPreferdPlacementIndex()];
                            if (FeasiblePlacement::checkCollision(&placement, &otherFP,
                                                                  Physics::getINSTANCE().getBoard()->getTileHeight())) {
                                found = true;
                                break;
                            }
                        }

                        if (found)
                            continue;

                        unsigned int score = regions[m].evaluatePlacement(placement, false);

                        if (score < bestScore) {
                            bestScore = score;
                            index = i;
                        }
                    }

                    if (bestScore < nowScore) {
                        //Found a better placement
                        FeasiblePlacement newPlacement = regions[m].getFeasiblePlacements()[index];

                        Point2D boardDim = Physics::getINSTANCE().getBoard()->getDimension();
                        Vector2 minusHalfBoardDim = Vector2(-(float) boardDim.get_y() / 2,
                                                            -(float) boardDim.get_x() / 2);

                        Vector2 regionPos = Vector2(newPlacement.getStartPosition().get_x(),
                                                    newPlacement.getStartPosition().get_y());
                        regionPos.add(minusHalfBoardDim);

                        Vector2 halfDim = Vector2((float) newPlacement.getDimension().get_x() * 0.5,
                                                  (float) newPlacement.getDimension().get_y() * 0.5);
                        regionPos.add(halfDim);

                        regions[m].getRb()->setPosition(regionPos);
                        regions[m].getRb()->setDimension(
                                Vector2(newPlacement.getDimension().get_x(), newPlacement.getDimension().get_y()));

                        regions[m].setPreferdPlacementIndex(index);
                        regions[m].setPreferedAnchorPoint(regionPos);

                        improved = true;
                    }


                }
            }
        }else{
            for (int i = 0; i < regionNum; ++i) {
                regions[i].setRegionState(PhysicsRegionState::FLOATING);
            }
        }

        //Initial evaluation
        for (int k = 0; k < regionNum; ++k) {
            if(regions[k].getRegionState() == PhysicsRegionState::PLACED)
                continue;

            regions[k].evaluatePlacementAndShape(false);
            regions[k].getRb()->setPosition(regions[k].getPreferedAnchorPoint());
            regions[k].getRb()->setSpeed(Vector2(0,0));
        }

        state = FloortplanningMangerState ::SEARCH_PLACEM;
        _time = 0;
    }
}

Problem *FloorplanningManager::getProblem() const {
    return problem;
}

void FloorplanningManager::setProblem(Problem *problem) {
    FloorplanningManager::problem = problem;
}

FloortplanningMangerState FloorplanningManager::getState() const {
    return state;
}

void FloorplanningManager::setStartTime(time_t startTime) {
    FloorplanningManager::startTime = startTime;
}
