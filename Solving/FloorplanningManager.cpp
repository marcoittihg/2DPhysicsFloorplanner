//
// Created by Marco on 31/12/17.
//

#include <thread>
#include <iostream>
#include <cmath>
#include <set>
#include "FloorplanningManager.h"
#include "PhysicsRegion.h"
#include "FloortplanningMangerState.h"

void FloorplanningManager::start() {
    time = 0;
    wireStabTime = 400.0;
    closestAlternativeRefreshTime = 80.0;
    lastAlternativeRefreshTime = 0.0;

    placemStabTime = 60.0f;
    state = FloortplanningMangerState::START;
}

void FloorplanningManager::onPysicsStep() {
    time += Physics::getINSTANCE().getFIXED_STEP_TIME();

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

                float radius = 1.15 * std::sqrt(blockNum / M_PI);

                regions[i].getRb()->setDimension(Vector2(radius,radius));
                floatingReg++;
            }
        }

        if(floatingReg == 0) {
            state = FloortplanningMangerState::END;

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

            //Print the solution
            std::cout<<"Solution: "<<std::endl;
            for (int i = 0; i < regionNum; ++i) {
                PhysicsRegion region = regions[i];
                FeasiblePlacement fp = region.getFeasiblePlacements()[region.getPreferdPlacementIndex()];

                std::cout<<fp.getStartPosition().get_x()+1<<" "<< fp.getStartPosition().get_y()+1<<" "<<+fp.getDimension().get_x()<<" "<<+fp.getDimension().get_y()<<std::endl;
            }
            return;
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
            time = 0;
            Physics::getINSTANCE().setNoiseSpeedCoeff(10);
            Physics::getINSTANCE().setNoiseModulus(0);
            Physics::getINSTANCE().setWireForceCoeff(0.0);
        }else{
            //Change state for next iteration
            state = FloortplanningMangerState::WAITING_FOR_WIRE_STABILITY;
            Physics::getINSTANCE().setWireForceCoeff(1);
            Physics::getINSTANCE().setEnableBarrierCollisions(true);
            time = 0;
            Physics::getINSTANCE().setIoForceMultiplier(1);
            Physics::getINSTANCE().setEnableRegionCollisions(false);
            Physics::getINSTANCE().setFIXED_STEP_TIME(0.01);
        }
    } else if(state == FloortplanningMangerState::WAITING_FOR_WIRE_STABILITY){
        PhysicsRegion* regions = Physics::getINSTANCE().getPhysicsRegions();
        int regionNum = problem->getNumRegions();


        if(time > 20)
            Physics::getINSTANCE().setSeparationCoeff(30*(time-20));

        if(time > wireStabTime){
            //Save wire stability position for each floating region
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


            //Go to search placement state
            time = 0;
            state = FloortplanningMangerState::SEARCH_PLACEM;
            Physics::getINSTANCE().setNoiseSpeedCoeff(10);
            Physics::getINSTANCE().setNoiseModulus(0);
            Physics::getINSTANCE().setWireForceCoeff(0.0);
            Physics::getINSTANCE().setEnableBarrierCollisions(false);
            Physics::getINSTANCE().setIoForceMultiplier(1);
            Physics::getINSTANCE().setEnableRegionCollisions(true);
            Physics::getINSTANCE().setFIXED_STEP_TIME(0.01);
            Physics::getINSTANCE().setLinearDrag(0.1);
        }

    }else if(state == FloortplanningMangerState::SEARCH_PLACEM){
        if(time > 40)
            Physics::getINSTANCE().setSeparationCoeff((time-40));
        else{
            Physics::getINSTANCE().setSeparationCoeff(0);
        }
        Physics::getINSTANCE().setPreferedAnchorCoeff(5*time);
        //Physics::getINSTANCE().setClosestAnchorCoeff(sqrt(time));
        //Physics::getINSTANCE().setNoiseModulus(sqrt(time));


        //If is passed enough time select the region that need to reevaluate his prefered choice
        PhysicsRegion* regions = Physics::getINSTANCE().getPhysicsRegions();
        int regionNum = problem->getNumRegions();
        if(time - lastAlternativeRefreshTime > closestAlternativeRefreshTime) {

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
                state = FloortplanningMangerState ::START;
            }
            time = 0;
        }

        /*
        if( time < placemStabTime)
            return;

        //Evaluate if the placement is flaoting or placed
        //A placement is placed if its distance from the prefered placing is close to 0
        //If all the regions are placed the solution is found
        int floatingCounter = 0;
        for (int j = 0; j < regionNum; ++j) {
            bool found = false;
            for (int i = 0; i < regionNum; ++i) {
                if(i == j)
                    continue;
                //The region must not overlap with placements of other regions
                if(FeasiblePlacement::checkCollision(
                        &regions[j].getFeasiblePlacements()[regions[j].getPreferdPlacementIndex()],
                        &regions[i].getFeasiblePlacements()[regions[i].getPreferdPlacementIndex()],
                        Physics::getINSTANCE().getBoard()->getTileHeight()
                        )
                        ){
                    found = true;
                    break;
                }
            }
            if(found){
                regions[j].setRegionState(PhysicsRegionState::FLOATING);
                floatingCounter++;
                continue;
            }

            Vector2 minusAchorPos = regions[j].getPreferedAnchorPoint();
            minusAchorPos.multiply(-1);

            minusAchorPos.add(regions[j].getRb()->getPosition());

            float distance = minusAchorPos.sqrMagnitude();

            if(distance < 0.001){
                regions[j].setRegionState(PhysicsRegionState::PLACED);
            }else{
                regions[j].setRegionState(PhysicsRegionState::FLOATING);
                floatingCounter++;
            }
        }

        if(floatingCounter == 0){
            //All the regions are placed
            time = 0;
            state = FloortplanningMangerState::START;
        }


        */
        //Check if one region meet the requirements to take it's prefered placement
        /*
        PhysicsRegion* regions = Physics::getINSTANCE().getPhysicsRegions();
        int regionNum = problem->getNumRegions();

        for (int j = 0; j < regionNum; ++j) {
            //for each region check if there is an other region that
            //share the prefered placement area for at most n%
            //If it is not the region is placed
            if(regions[j].getRegionState() == PhysicsRegionState::PLACED)
                continue;

            int prefIndex = regions[j].getPreferdPlacementIndex();

            FeasiblePlacement fp = regions[j].getFeasiblePlacements()[prefIndex];

            //placement point2d to float
            Vector2 placementPos, placementDim;
            Vector2 oldPlacementPos, oldPlacementDim;

            Point2D boardDim = Physics::getINSTANCE().getBoard()->getDimension();
            Vector2 minusHalfBoardDim = Vector2(-(float)boardDim.get_y() / 2, -(float)boardDim.get_x() / 2);

            placementPos = Vector2(fp.getStartPosition().get_x(),fp.getStartPosition().get_y());
            placementDim = Vector2(fp.getDimension().get_x(),fp.getDimension().get_y());

            placementPos.add(minusHalfBoardDim);

            float placementTotArea = placementDim.getX() * placementDim.getY();

            oldPlacementPos = placementPos;
            oldPlacementDim = placementDim;

            Rigidbody* rbRegion = regions[j].getRb();

            placementPos.setX( std::max(placementPos.getX(), rbRegion->getPosition().getX() - rbRegion->getDimension().getX() * (float)0.5));
            placementPos.setY( std::max(placementPos.getY(), rbRegion->getPosition().getY() - rbRegion->getDimension().getY() * (float)0.5));

            placementDim.setX( std::min(oldPlacementPos.getX() + oldPlacementDim.getX(), rbRegion->getPosition().getX()+ rbRegion->getDimension().getX()* (float)0.5));
            placementDim.setY( std::min(oldPlacementPos.getY() + oldPlacementDim.getY(), rbRegion->getPosition().getY()+ rbRegion->getDimension().getY()* (float)0.5));

            placementDim.setX(placementDim.getX() - placementPos.getX());
            placementDim.setY(placementDim.getY() - placementPos.getY());

            if(placementDim.getX() * placementDim.getY() / placementTotArea < 0.8)
                continue;   //The area itself do not cover the 80% of the placement so it is impossible to take the placement


            bool found = false;
            for (int i = 0; i < regionNum; ++i) {
                if(i == j)  //Do not check with itself
                    continue;

                Vector2 overlappArea;
                Vector2 overlappAreaStop;
                Vector2 regPos = regions[i].getRb()->getPosition();
                Vector2 regDim = regions[i].getRb()->getDimension();

                overlappArea.setX(std::max(placementPos.getX(), regPos.getX() - regDim.getX()* (float)0.5));
                overlappArea.setY(std::max(placementPos.getY(), regPos.getY() - regDim.getY()* (float)0.5));

                overlappAreaStop.setX(std::min(placementPos.getX() + placementDim.getX(), regPos.getX() + regDim.getX()* (float)0.5));
                overlappAreaStop.setY(std::min(placementPos.getY() + placementDim.getY(), regPos.getY() + regDim.getY()* (float)0.5));

                float overArea = (overlappAreaStop.getX() - overlappArea.getX()) * (overlappAreaStop.getY() - overlappArea.getY());

                float newFreeArea = placementTotArea - overArea;

                if(newFreeArea / placementTotArea < 0.8){
                    //Placement found
                    found = true;
                    break;
                }
            }
            if(!found && time > placemStabTime){
                //std::cout<<"P Found:"<< j<<std::endl;
                regions[j].setRegionState(PhysicsRegionState::PLACED);
                time = 0;
                state = FloortplanningMangerState::START;
            }

        }*/
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
