//
// Created by Marco on 28/12/17.
//

#include "PhysicsRegion.h"
#include "FloorplanningManager.h"
#include <cmath>
#include <iostream>
#include <random>

FeasiblePlacement *PhysicsRegion::getFeasiblePlacements() const {
    return feasiblePlacements;
}

void PhysicsRegion::setFeasiblePlacements(FeasiblePlacement *feasiblePlacements) {
    PhysicsRegion::feasiblePlacements = feasiblePlacements;
}

unsigned short PhysicsRegion::getPreferdPlacementIndex() const {
    return preferdPlacementIndex;
}

void PhysicsRegion::setPreferdPlacementIndex(unsigned short preferdPlacementIndex) {
    PhysicsRegion::preferdPlacementIndex = preferdPlacementIndex;
}

unsigned char PhysicsRegion::getRegionIndex() const {
    return regionIndex;
}

void PhysicsRegion::setRegionIndex(unsigned char regionIndex) {
    PhysicsRegion::regionIndex = regionIndex;
}

const std::vector<PhysicsRegion *> &PhysicsRegion::getInterconnectedRegions() const {
    return interconnectedRegions;
}

void PhysicsRegion::setInterconnectedRegions(const std::vector<PhysicsRegion *> &interconnectedRegions) {
    PhysicsRegion::interconnectedRegions = interconnectedRegions;
}

void PhysicsRegion::onPhysicsStart() {

}

void PhysicsRegion::fixedPhysicsStep() {
    Point2D boardDim = Physics::getINSTANCE().getBoard()->getDimension();
    Vector2 halfBoardDim = Vector2((float)boardDim.get_y() / (float)2, (float)boardDim.get_x() / (float)2);
    halfBoardDim.multiply(-1);

    Vector2 pos = rb->getPosition();

    pos.multiply(-1);

    if(regionType == PhysicsRegionType::IO_INT || regionType == PhysicsRegionType::NOIO_INT) {
        //IF the region have interc

        for (int i = 0; i < interconnectedRegions.size(); ++i) {
            //For each interconnected regions apply the spring force
            Vector2 intercPos = interconnectedRegions.at(i)->getRb()->getPosition();
            intercPos.add(pos);

            int numWire = interconnectedRegionsWeights.at(i);
            intercPos.multiply(Physics::getINSTANCE().getWireForceCoeff() * numWire);
            rb->addImpulse(intercPos, Physics::FIXED_STEP_TIME);
        }
    }

    if(regionType == PhysicsRegionType::IO_INT || regionType == PhysicsRegionType::IO_NOINT) {
        //If the region have IOs

        for (int j = 0; j < IONum; ++j) {
            //For each IO apply the force
            Vector2 IOPos = Vector2(regionIO[j].getPortColumn(), regionIO[j].getPortRow());
            IOPos.add(halfBoardDim);

            int numWire = regionIO[j].getNumWires();

            IOPos.add(pos);

            IOPos.multiply(Physics::getINSTANCE().getWireForceCoeff() * numWire * Physics::getINSTANCE().getIoForceMultiplier());

            rb->addImpulse(IOPos, Physics::FIXED_STEP_TIME);
        }
    }


    //Add best and closest anchor forces
    Vector2 bestAnchor = preferedAnchorPoint;
    bestAnchor.add(pos);

    //Calculate distance from pref anchor
    float dist = bestAnchor.mangnitude() + 1;


    bestAnchor.multiply(Physics::getINSTANCE().getPreferedAnchorCoeff() * anchorForceMultiplier * scoreImpactMultiplier / dist);

    Vector2 force = bestAnchor;
    //force.add(altAnchor);

    rb->addImpulse(force, Physics::FIXED_STEP_TIME);
}

PhysicsRegion::PhysicsRegion() {

}

Rigidbody *PhysicsRegion::getRb() const {
    return rb;
}

void PhysicsRegion::setRb(Rigidbody *rb) {
    PhysicsRegion::rb = rb;
}

PhysicsRegionState PhysicsRegion::getRegionState() const {
    return regionState;
}

void PhysicsRegion::setRegionState(PhysicsRegionState regionState) {
    PhysicsRegion::regionState = regionState;
}

Vector2 PhysicsRegion::getPreferedAnchorPoint() {
    return preferedAnchorPoint;
}

void PhysicsRegion::setPreferedAnchorPoint(const Vector2 &preferedAnchorPoint) {
    PhysicsRegion::preferedAnchorPoint = preferedAnchorPoint;
}


void PhysicsRegion::addInterconnectedRegion(PhysicsRegion *newRegion, int weight) {
    interconnectedRegionsWeights.push_back(weight);
    interconnectedRegions.push_back(newRegion);
}

PhysicsRegionType PhysicsRegion::getRegionType() const {
    return regionType;
}

void PhysicsRegion::setRegionType(PhysicsRegionType regionType) {
    PhysicsRegion::regionType = regionType;
}

char PhysicsRegion::getIONum() const {
    return IONum;
}

void PhysicsRegion::setIONum(char IONum) {
    PhysicsRegion::IONum = IONum;
}

RegionIOData *PhysicsRegion::getRegionIO() const {
    return regionIO;
}

void PhysicsRegion::setRegionIO(RegionIOData *regionIO) {
    PhysicsRegion::regionIO = regionIO;
}

void PhysicsRegion::resetPositionAndShape() {
    rb->setPosition(Vector2(0,0));
    rb->setSpeed(Vector2(0,0));
    rb->setDimension(Vector2(6,6));
}

unsigned int PhysicsRegion::evaluatePlacement(FeasiblePlacement fp){

    Point2D boardDim = Physics::getINSTANCE().getBoard()->getDimension();
    Vector2 minusHalfBoardDim = Vector2(-(float)boardDim.get_y() / 2, -(float)boardDim.get_x() / 2);

    float placementMidX = static_cast<float>(fp.getStartPosition().get_x() + 0.5 * fp.getDimension().get_x());
    float placementMidY = static_cast<float>(fp.getStartPosition().get_y() + 0.5 * fp.getDimension().get_y());

    int wireWeight = FloorplanningManager::getINSTANCE().getProblem()->getWireCost();
    int areaWeight = FloorplanningManager::getINSTANCE().getProblem()->getAreaCost();

    //Set the base score loss as the area score loss
    unsigned int scoreLoss = fp.getAreaCost() * areaWeight;

    //Add the score loss of each IO
    for (int j = 0; j < IONum; ++j) {
        float xDist = abs(placementMidX - (float) regionIO[j].getPortColumn());
        float yDist = abs(placementMidY - (float) regionIO[j].getPortRow());

        scoreLoss += (xDist + yDist) * regionIO[j].getNumWires() * wireWeight;
    }

    //Add the score loss of each interconnected region
    for (int j = 0; j < interconnectedRegions.size(); ++j) {
        PhysicsRegion* intReg = interconnectedRegions.at(j);

        float regX, regY;

        if(false){
            //Get as placement point the actual placed point
            unsigned short placementIndex = intReg->getPreferdPlacementIndex();
            FeasiblePlacement intPlacem = intReg->getFeasiblePlacements()[placementIndex];

            regX = static_cast<float>(intPlacem.getStartPosition().get_x() + 0.5 * intPlacem.getDimension().get_x());
            regY = static_cast<float>(intPlacem.getStartPosition().get_y() + 0.5 * intPlacem.getDimension().get_y());
        }else{
            //Get as placement point the point of wire stabilit
            regX = intReg->getWireStabilityPosition().getX();
            regY = intReg->getWireStabilityPosition().getY();
        }

        float xDist = abs(placementMidX - regX + minusHalfBoardDim.getX());
        float yDist = abs(placementMidY - regY + minusHalfBoardDim.getY());

        scoreLoss += (xDist + yDist) * interconnectedRegionsWeights.at(j) * wireWeight;
    }

    return scoreLoss;
}

void PhysicsRegion::evaluatePlacementAndShape(bool isStart) {

    PhysicsRegion* regions = Physics::getINSTANCE().getPhysicsRegions();
    int regNum = Physics::getINSTANCE().getRegionNum();

    Point2D boardDim = Physics::getINSTANCE().getBoard()->getDimension();
    Vector2 minusHalfBoardDim = Vector2(-(float)boardDim.get_y() / 2, -(float)boardDim.get_x() / 2);

    for (int i = 0; i < placementNum; ++i) {
        //For each placement
        unsigned int scoreLoss = evaluatePlacement(feasiblePlacements[i]);

        feasiblePlacements[i].setScoreLoss(scoreLoss);
    }

    struct ScoreComparer{
        bool operator()(FeasiblePlacement f1, FeasiblePlacement f2){ return f1.getScoreLoss() < f2.getScoreLoss();}
    } scoreComparer;

    std::sort(feasiblePlacements, feasiblePlacements + placementNum, scoreComparer);

    std::vector<unsigned short int> minIndexes;
    unsigned int minScoreLoss = std::numeric_limits<unsigned int>::max();

    for (int i = 0; i < placementNum; ++i) {
        //Check if the placement overlapp with a previous taken region
        bool found = false;
        for (int k = 0; k < regNum; ++k) {
            if(regions[k].getRegionState() == PhysicsRegionState::PLACED) {
                if (k == regionIndex || isStart)
                    continue;

                int takenPlacementIndex = regions[k].getPreferdPlacementIndex();
                if (FeasiblePlacement::checkCollision(
                        &feasiblePlacements[i],
                        &regions[k].getFeasiblePlacements()[takenPlacementIndex],
                        FloorplanningManager::getINSTANCE().getProblem()->getBoard()->getTileHeight())) {

                    found = true;
                    break;
                }
            }
        }

        if(found)
            continue;

        if(feasiblePlacements[i].getScoreLoss() < minScoreLoss){
            minScoreLoss = feasiblePlacements[i].getScoreLoss();
            minIndexes.clear();
            minIndexes.push_back(i);

        }else if(feasiblePlacements[i].getScoreLoss() == minScoreLoss){
            minIndexes.push_back(i);
        }
    }


    std::random_device r;
    std::ranlux48_base e1(r());
    std::uniform_int_distribution<int> uniform_dist(0, RAND_MAX);

    unsigned short int index;

    if(minIndexes.empty()){
        //No placement found, need to select it in an other way
        struct Resources{
        public:
            unsigned short CLB = 0;
            unsigned short BRAM = 0;
            unsigned short DSP = 0;
        };

        Resources res[regNum];

        Board* board = Physics::getINSTANCE().getBoard();

        for (int j = 0; j < regNum; ++j) {
            //For each region prefered placement evaluate the wasted resources
            FeasiblePlacement fp = regions[j].getFeasiblePlacements()[regions[j].getPreferdPlacementIndex()];


            for (int i = 0; i < fp.getDimension().get_x(); ++i) {
                for (int k = 0; k < fp.getDimension().get_y(); ++k) {
                    Block block = board->getBlockMatrix(fp.getStartPosition().get_y() + k, fp.getStartPosition().get_x() + i);

                    switch(block){
                        case Block::CLB_BLOCK : res[j].CLB++;
                            break;
                        case Block::DSP_BLOCK : res[j].DSP++;
                            break;
                        case Block::BRAM_BLOCK: res[j].BRAM++;
                            break;
                    }
                }
            }

            ProblemRegion* region = const_cast<ProblemRegion *>(FloorplanningManager::getINSTANCE().getProblem()->getFloorplanProblemRegion(j));
            res[j].CLB -= region->getCLBNum();
            res[j].BRAM -= region->getBRAMNum();
            res[j].DSP -= region->getDSPNum();
        }


        for (int i = 0; i < placementNum; ++i) {
            //For each placement
            FeasiblePlacement fp = feasiblePlacements[i];

            float fpMidX = static_cast<float>(fp.getStartPosition().get_x() + 0.5 * fp.getDimension().get_x());
            float fpMidY = static_cast<float>(fp.getStartPosition().get_y() + 0.5 * fp.getDimension().get_y());

            Vector2 distance = Vector2(fpMidX, fpMidY);
            distance.multiply(-1);
            distance.add(regions[regionIndex].getWireStabilityPosition());

            Resources resources;
            resources.DSP = resources.CLB = resources.BRAM = 0;

            //Evaluate the waste of resources
            for (int i = 0; i < fp.getDimension().get_x(); ++i) {
                for (int k = 0; k < fp.getDimension().get_y(); ++k) {
                    Block block = board->getBlockMatrix(fp.getStartPosition().get_y() + k, fp.getStartPosition().get_x() + i);

                    switch(block){
                        case Block::CLB_BLOCK : resources.CLB++;
                            break;
                        case Block::DSP_BLOCK : resources.DSP++;
                            break;
                        case Block::BRAM_BLOCK: resources.BRAM++;
                            break;
                    }
                }
            }

            ProblemRegion* region =
                    const_cast<ProblemRegion *>(
                            FloorplanningManager::getINSTANCE().getProblem()->getFloorplanProblemRegion(regionIndex)
                    );

            resources.CLB -= region->getCLBNum();
            resources.BRAM -= region->getBRAMNum();
            resources.DSP -= region->getDSPNum();

            //Find the regions that the placement overlap with
            std::vector<int> regIndexes;
            for (int j = 0; j < regNum; ++j) {
                FeasiblePlacement regPlac = regions[j].getFeasiblePlacements()[regions[j].getPreferdPlacementIndex()];

                if(FeasiblePlacement::checkCollision(&fp, &regPlac, board->getTileHeight())){
                    regIndexes.push_back(j);
                }
            }

            std::vector<int> waste;
            //For each region count the difference of wasted resources
            for (int l = 0; l < regIndexes.size(); ++l) {
                int regionIndex = regIndexes.at(l);
                int w = res[regionIndex].DSP - resources.DSP
                        + res[regionIndex].BRAM - resources.BRAM
                        + res[regionIndex].CLB - resources.CLB;

                if(res[regionIndex].DSP >= region->getDSPNum()){
                    w*=2;
                }
                if(res[regionIndex].CLB >= region->getCLBNum()){
                    w*=2;
                }
                if(res[regionIndex].BRAM >= region->getBRAMNum()){
                    w*=2;
                }
                w *= 1/regions[regionIndex].getScoreImpactMultiplier();


                waste.push_back(w);
            }

            //Calculate the average waste
            int avgWaste = 0;
            for (int m = 0; m < waste.size(); ++m) {
                avgWaste+=waste.at(m);
            }
            for (int l = 0; l < regIndexes.size(); ++l) {
                int regionIndex = regIndexes.at(l);
                if(regions[regionIndex].getRegionState() == PhysicsRegionState::PLACED)
                    avgWaste /= 2;
            }

            avgWaste /= (int)waste.size();
            avgWaste = avgWaste <= 0 ? 1 : avgWaste;
            unsigned int a = static_cast<unsigned int>(avgWaste);
            feasiblePlacements[i].setScoreLoss(a);
        }

        //Count the total waste
        unsigned int totWaste = 0;
        for (int n = 0; n < placementNum; ++n) {
            FeasiblePlacement fp = feasiblePlacements[n];
            totWaste += fp.getScoreLoss();
        }

        //Select the placement
        int randWaste = rand() % totWaste;

        int cumulativeWaste = 0;
        for (unsigned short i1 = 0; i1 < placementNum; ++i1) {
            cumulativeWaste+= feasiblePlacements[i1].getScoreLoss();
            if(cumulativeWaste >= randWaste){
                //Found the index
                index = i1;
                break;
            }
        }
    }else {
        index = minIndexes.at(uniform_dist(e1) % minIndexes.size());
    }

    preferdPlacementIndex = index;
    FeasiblePlacement prefPlacemnt = feasiblePlacements[index];

    Vector2 stabPoint = wireStabilityPosition;
    stabPoint.multiply(-1);
    float pMidX = static_cast<float>(prefPlacemnt.getStartPosition().get_x() + 0.5 * prefPlacemnt.getDimension().get_x());
    float pMidY = static_cast<float>(prefPlacemnt.getStartPosition().get_y() + 0.5 * prefPlacemnt.getDimension().get_y());

    stabPoint.add(Vector2(pMidX, pMidY));

    anchorForceMultiplier = 1 + 1000 * (1 - (float)index / (float)placementNum) / stabPoint.mangnitude();


    preferedAnchorPoint = Vector2(
            prefPlacemnt.getStartPosition().get_x() + prefPlacemnt.getDimension().get_x() * 0.5,
            prefPlacemnt.getStartPosition().get_y() + prefPlacemnt.getDimension().get_y() * 0.5
    );


    preferedAnchorPoint.add(minusHalfBoardDim);


    //Set shape
    rb->setDimension(Vector2(prefPlacemnt.getDimension().get_x(),prefPlacemnt.getDimension().get_y()));

    /*std::cout<<"Region "<<+regionIndex<<std::endl;
    std::cout<<"Start placement "<<+prefPlacemnt.getStartPosition().get_x()<<" , "<<+prefPlacemnt.getStartPosition().get_y()<<std::endl;
    std::cout<<"Dim placement "<<+prefPlacemnt.getDimension().get_x()<<" , "<<+prefPlacemnt.getDimension().get_y()<<std::endl;
    std::cout<<"Anchor point"<<preferedAnchorPoint.getX()<<" , "<<preferedAnchorPoint.getX()<<std::endl<<std::endl;
*/
}

void PhysicsRegion::savePositionAsWireStability() {
    wireStabilityPosition = rb->getPosition();
}

const Vector2 &PhysicsRegion::getWireStabilityPosition() const {
    return wireStabilityPosition;
}

unsigned short PhysicsRegion::getPlacementNum() const {
    return placementNum;
}

void PhysicsRegion::setPlacementNum(unsigned short placementNum) {
    PhysicsRegion::placementNum = placementNum;
}

float PhysicsRegion::getScoreImpactMultiplier() const {
    return scoreImpactMultiplier;
}

void PhysicsRegion::setScoreImpactMultiplier(float scoreImpactMultiplier) {
    PhysicsRegion::scoreImpactMultiplier = scoreImpactMultiplier;
}
