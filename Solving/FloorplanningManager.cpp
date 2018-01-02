//
// Created by Marco on 31/12/17.
//

#include <thread>
#include <iostream>
#include <cmath>
#include "FloorplanningManager.h"
#include "PhysicsRegion.h"
#include "FloortplanningMangerState.h"

void FloorplanningManager::start() {
    time = 0;
    wireStabTime = 40.0;
    closestAlternativeRefreshTime = 80.0;
    lastAlternativeRefreshTime = 0.0;

    placemStabTime = 30.0f;
    state = FloortplanningMangerState::START;
}

void FloorplanningManager::onPysicsStep() {
    time += Physics::FIXED_STEP_TIME;

    if(state == FloortplanningMangerState::START){
        //Check how many floating regions there are
        int floatingReg = 0;

        PhysicsRegion* regions = Physics::getINSTANCE().getPhysicsRegions();
        int regionNum = problem->getNumRegions();

        for (int i = 0; i < regionNum; ++i) {
            if(regions[i].getRegionState() == PhysicsRegionState::FLOATING) {
                regions[i].resetPositionAndShape();
                floatingReg++;
            }
        }

        if(floatingReg == 0) {
            state = FloortplanningMangerState::END;

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

        if(problem->getWireCost() == 0){
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
            Physics::getINSTANCE().setWireForceCoeff(0.5);
            Physics::getINSTANCE().setEnableBarrierCollisions(true);
            time = 0;
            Physics::getINSTANCE().setIoForceMultiplier(20);
        }
    } else if(state == FloortplanningMangerState::WAITING_FOR_WIRE_STABILITY){
        PhysicsRegion* regions = Physics::getINSTANCE().getPhysicsRegions();
        int regionNum = problem->getNumRegions();
        /*
        Point2D boardDim = Physics::getINSTANCE().getBoard()->getDimension();
        Vector2 halfBoardDim = Vector2(boardDim.get_y()/2, boardDim.get_x()/2);

        Vector2 topLeft = Vector2(-halfBoardDim.getX() *2/3 , halfBoardDim.getY() / 2);
        Vector2 topRight = Vector2(halfBoardDim.getX() *2/3 , halfBoardDim.getY() / 2);
        Vector2 bottomLeft = Vector2(-halfBoardDim.getX() *2/3 , -halfBoardDim.getY() / 2);
        Vector2 bottomRight = Vector2(halfBoardDim.getX() *2/3 , -halfBoardDim.getY() / 2);
        Vector2 top = Vector2(0 , halfBoardDim.getY() *2/ 3);
        Vector2 center = Vector2(0 , 0);
        Vector2 bottom = Vector2(0, -halfBoardDim.getY() *2/ 3);

        Vector2 regionTopLeft = regions[16].getRb()->getPosition();
        Vector2 regionTopRight = regions[15].getRb()->getPosition();
        Vector2 regionBottomLeft = regions[1].getRb()->getPosition();
        Vector2 regionBottomRight = regions[12].getRb()->getPosition();
        Vector2 regionTop = regions[25].getRb()->getPosition();
        Vector2 regionCenter = regions[28].getRb()->getPosition();
        Vector2 regionBottom = regions[19].getRb()->getPosition();

        regionTopLeft.multiply(-1);
        regionTopRight.multiply(-1);
        regionBottomLeft.multiply(-1);
        regionBottomRight.multiply(-1);
        regionTop.multiply(-1);
        regionCenter.multiply(-1);
        regionBottom.multiply(-1);

        Vector2 force1 = topLeft;
        Vector2 force2 = topRight;
        Vector2 force3 = bottomLeft;
        Vector2 force4 = bottomRight;
        Vector2 force5 = top;
        Vector2 force6 = center;
        Vector2 force7 = bottom;

        force1.add(regionTopLeft);
        force2.add(regionTopRight);
        force3.add(regionBottomLeft);
        force4.add(regionBottomRight);
        force5.add(regionTop);
        force6.add(regionCenter);
        force7.add(regionBottom);

        force1.multiply(500);
        force2.multiply(500);
        force3.multiply(500);
        force4.multiply(500);
        force5.multiply(500);
        force6.multiply(500);
        force7.multiply(500);

        regions[16].getRb()->addImpulse(force1, Physics::FIXED_STEP_TIME);
        regions[15].getRb()->addImpulse(force2, Physics::FIXED_STEP_TIME);
        regions[1].getRb()->addImpulse(force3, Physics::FIXED_STEP_TIME);
        regions[12].getRb()->addImpulse(force4, Physics::FIXED_STEP_TIME);
        regions[25].getRb()->addImpulse(force5, Physics::FIXED_STEP_TIME);
        regions[28].getRb()->addImpulse(force6, Physics::FIXED_STEP_TIME);
        regions[19].getRb()->addImpulse(force7, Physics::FIXED_STEP_TIME);
        */

        if(time > 20)
            Physics::getINSTANCE().setSeparationCoeff(5*(time-20));

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

        }
    }else if(state == FloortplanningMangerState::SEARCH_PLACEM){
        if(time > 20)
            Physics::getINSTANCE().setSeparationCoeff((time-20));
        else{
            Physics::getINSTANCE().setSeparationCoeff(0);
        }
        Physics::getINSTANCE().setPreferedAnchorCoeff(10*sqrt(time));
        //Physics::getINSTANCE().setClosestAnchorCoeff(sqrt(time));
        //Physics::getINSTANCE().setNoiseModulus(sqrt(time));


        //If is passed enough time select the region that need to reevaluate his prefered choice
        PhysicsRegion* regions = Physics::getINSTANCE().getPhysicsRegions();
        int regionNum = problem->getNumRegions();
        float maxDistance = std::numeric_limits<float>::min();
        if(time - lastAlternativeRefreshTime > closestAlternativeRefreshTime) {
            int regionIndex = -1;
            for (int j = 0; j < regionNum; ++j) {
                if(regions[j].getRegionState() == PhysicsRegionState::PLACED)
                    continue;

                Vector2 minusAchorPos = regions[j].getPreferedAnchorPoint();
                minusAchorPos.multiply(-1);

                minusAchorPos.add(regions[j].getRb()->getPosition());

                float distance = minusAchorPos.mangnitude();

                if(maxDistance < distance){
                    maxDistance = distance;
                    regionIndex = j;
                }
            }

            if(regionIndex != -1) {
                regions[regionIndex].evaluatePlacementAndShape(false);
                time = 0;
                lastAlternativeRefreshTime = time;
            }
        }

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