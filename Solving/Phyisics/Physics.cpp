//
// Created by Marco on 28/12/17.
//

#include "Physics.h"
#include "../FloorplanningManager.h"
#include <math.h>

void Physics::onStart() {
    //Call the start method for each region of the simulation
    for (int i = 0; i < regionNum; ++i) {
        physicsRegions[i].onPhysicsStart();
    }


    noise = Vector2(0,0);
    noiseSpeedCoeff = 0;
    noiseModulus = 0;
    noiseStepDir = Vector2(0,1);
    FIXED_STEP_TIME = 0.02;
    enableBarrierCollisions = false;
    enableRegionCollisions = true;
}

void Physics::doStep() {
    //Call the before fixed step method for each region
    for (int i = 0; i < regionNum; ++i) {
        physicsRegions[i].fixedPhysicsStep();
    }

    //calculate collisions and apply separation forces
    for (int i = 0; i < regionNum && isEnableRegionCollisions(); ++i) {
        Rigidbody r1 = rigidbodies[i];
        FeasiblePlacement prefPlac1 = physicsRegions[i].getFeasiblePlacements()[physicsRegions[i].getPreferdPlacementIndex()];
        for (int j = i + 1; j < regionNum; ++j) {
            //For each distinct pair of rigidbodies
            Rigidbody r2 = rigidbodies[j];

            //Check if the prefered placements collid
            FeasiblePlacement prefPlac2 = physicsRegions[j].getFeasiblePlacements()[physicsRegions[j].getPreferdPlacementIndex()];

            if(!FeasiblePlacement::checkCollision(&prefPlac1, &prefPlac2, board->getTileHeight()))
                continue;

            //Check if there is a collision
            float xDist = r1.getPosition().getX() - r2.getPosition().getX();
            float yDist = r1.getPosition().getY() - r2.getPosition().getY();

            xDist = xDist < 0 ? -xDist : xDist;
            yDist = yDist < 0 ? -yDist : yDist;

            xDist -= (r1.getDimension().getX() + r2.getDimension().getX()) / 2;
            yDist -= (r1.getDimension().getY() + r2.getDimension().getY()) / 2;

            if(xDist >= 0 || yDist >= 0)
                continue;

            //Collision
            Vector2 force;
            if(xDist < yDist){
                //Push y
                yDist = -yDist;
                force.setX(0);

                if(r1.getPosition().getY() > r2.getPosition().getY()){
                    force.setY(yDist * separationCoeff);
                }else{
                    force.setY(-yDist * separationCoeff);
                }
            }else{
                //Push x
                xDist = -xDist;
                force.setY(0);

                if(r1.getPosition().getX() > r2.getPosition().getX()){
                    force.setX(xDist * separationCoeff);
                }else{
                    force.setX(-xDist * separationCoeff);
                }
            }


            //Apply separation forces
            rigidbodies[i].addImpulse(force);

            force.multiply(-1);
            rigidbodies[j].addImpulse(force);
        }
    }

    /*
    //Update and apply noise
    Vector2 noiseStep = noiseStepDir;
    noiseStep.multiply(noiseSpeedCoeff * FIXED_STEP_TIME);

    noise.add(noiseStep);

    float noiseMag = noise.sqrMagnitude();
    if(noiseMag > 1){
        noise.normalize();

        //Generate new noise step direction
        double randomAngle = static_cast<float> (rand()) / static_cast<float>(RAND_MAX) * 2 * M_PI;

        noiseStepDir = Vector2(cos(randomAngle), sin(randomAngle));
    }

    //Apply noise force
    Vector2 tmpNoise = noise;
    tmpNoise.multiply(noiseModulus);

    for (int i = 0; i < regionNum; ++i) {
        if(physicsRegions[i].getRegionState() == PhysicsRegionState::PLACED)
            continue;

        rigidbodies[i].addImpulse(tmpNoise, FIXED_STEP_TIME);
    }*/

    //Calculate maximum possible timeStep
    float maxDistance = std::numeric_limits<float>::max();

    for (int j = 0; j < regionNum; ++j) {
        Vector2 dim = rigidbodies[j].getDimension();
        float min = std::fmin(dim.getX(),dim.getY());
        if(min < maxDistance)
            maxDistance = min;
    }

    maxDistance /= 4;

    if(maxDistance > 0.1){
        maxDistance = 0.1;
    }

    FIXED_STEP_TIME = MAX_STEP_TIME;


    for (int j = 0; j < regionNum; ++j) {
        float lowerTimeBound = 0;
        float upperTimeBound = FIXED_STEP_TIME;

        Vector2 v0t = rigidbodies[j].getSpeed();
        v0t.multiply(upperTimeBound);

        Vector2 at2 = rigidbodies[j].getStepForce();
        at2.multiply(upperTimeBound * upperTimeBound);
        v0t.add(at2);

        float upperValue = v0t.mangnitude();

        if(upperValue < maxDistance)
            continue;

        for (int i = 0; i < 10; ++i) {
            float midTime = (lowerTimeBound+upperTimeBound) / 2;

            Vector2 v0t = rigidbodies[j].getSpeed();
            v0t.multiply(midTime);

            Vector2 at2 = rigidbodies[j].getStepForce();
            at2.multiply(midTime * midTime);

            v0t.add(at2);

            float midValue = v0t.mangnitude();
            midValue-=maxDistance;

            if(midValue > 0){
                upperTimeBound = midTime;
            }else{
                lowerTimeBound = midTime;
            }
        }
        FIXED_STEP_TIME = (upperTimeBound + lowerTimeBound) / 2;
    }

    if(FIXED_STEP_TIME == 0){
        FIXED_STEP_TIME = 0.01;
    }

    //Apply forces and move the regions
    for (int j = 0; j < regionNum; ++j) {
        rigidbodies[j].applyForces(FIXED_STEP_TIME);
        rigidbodies[j].applyLinearDrag(linearDrag);
        rigidbodies[j].applyMovement(FIXED_STEP_TIME);
        rigidbodies[j].resetStepForce();
    }

    //Check if some region is outside of the border
    if(enableBarrierCollisions) {
        for (int i = 0; i < regionNum; ++i) {
            Rigidbody &rb = rigidbodies[i];

            float halfWidth = static_cast<float>(rb.getDimension().getX() * 0.5);
            float halfHeight = static_cast<float>(rb.getDimension().getY() * 0.5);

            float boardWidth = board->getDimension().get_x();
            float boardHeight = board->getDimension().get_y();

            Vector2 halfBoardDim = Vector2(boardWidth, boardHeight);
            halfBoardDim.multiply(0.5);

            Vector2 pos = rb.getPosition();
            Vector2 spe = rb.getSpeed();


            if (pos.getX() + halfBoardDim.getX() < halfWidth) {
                pos.setX(halfWidth - halfBoardDim.getX());
                spe.setX(spe.getX() * (-barriersRestitutionCoeff));
            }
            if (pos.getY() + halfBoardDim.getY() < halfHeight) {
                pos.setY(halfHeight - halfBoardDim.getY());
                spe.setY(spe.getY() * (-barriersRestitutionCoeff));
            }

            if (pos.getX() > halfBoardDim.getX() - halfWidth) {
                pos.setX(halfBoardDim.getX() - halfWidth);
                spe.setX(spe.getX() * (-barriersRestitutionCoeff));
            }
            if (pos.getY() > halfBoardDim.getY() - halfHeight) {
                pos.setY(halfBoardDim.getY() - halfHeight);
                spe.setY(spe.getY() * (-barriersRestitutionCoeff));
            }

            rb.setPosition(pos);
            rb.setSpeed(spe);
        }
    }


    FloorplanningManager::getINSTANCE().onPysicsStep();
}

PhysicsRegion* Physics::addRegion() {

    Rigidbody* newRB = new Rigidbody[regionNum + 1];
    PhysicsRegion* newReg = new PhysicsRegion[regionNum + 1];

    regionNum++;

    for (int i = 0; i < regionNum; ++i) {
        newReg[i].setRb(&newRB[i]);

        if(i == regionNum - 1)
            continue;

        newRB[i].setPosition(rigidbodies[i].getPosition());
        newRB[i].setSpeed(rigidbodies[i].getSpeed());
        newRB[i].setDimension(rigidbodies[i].getDimension());

        newReg[i].setRegionIndex(physicsRegions[i].getRegionIndex());
        newReg[i].setRegionState(physicsRegions[i].getRegionState());
        newReg[i].setPreferedAnchorPoint(physicsRegions[i].getPreferedAnchorPoint());
        newReg[i].setRegionType(physicsRegions[i].getRegionType());
        newReg[i].setRegionState(physicsRegions[i].getRegionState());
        newReg[i].setInterconnectedRegions(physicsRegions[i].getInterconnectedRegions());
        newReg[i].setFeasiblePlacements(physicsRegions[i].getFeasiblePlacements());
        newReg[i].setIONum(physicsRegions[i].getIONum());
        newReg[i].setRegionIO(physicsRegions[i].getRegionIO());
        newReg[i].setPlacementNum(physicsRegions[i].getPlacementNum());
    }

    delete[] rigidbodies;
    delete[] physicsRegions;

    rigidbodies = newRB;
    physicsRegions = newReg;

    return &newReg[regionNum - 1];
}

Board *Physics::getBoard() const {
    return board;
}

void Physics::setBoard(Board *board) {
    Physics::board = board;
}

PhysicsRegion *Physics::getPhysicsRegions() const {
    return physicsRegions;
}

unsigned short Physics::getRegionNum() const {
    return regionNum;
}

float Physics::getPreferedAnchorCoeff() const {
    return preferedAnchorCoeff;
}

float Physics::getClosestAnchorCoeff() const {
    return closestAnchorCoeff;
}

float Physics::getWireForceCoeff() const {
    return wireForceCoeff;
}

void Physics::setSeparationCoeff(float separationCoeff) {
    Physics::separationCoeff = separationCoeff;
}

void Physics::setPreferedAnchorCoeff(float preferedAnchorCoeff) {
    Physics::preferedAnchorCoeff = preferedAnchorCoeff;
}

void Physics::setClosestAnchorCoeff(float closestAnchorCoeff) {
    Physics::closestAnchorCoeff = closestAnchorCoeff;
}

void Physics::setWireForceCoeff(float wireForceCoeff) {
    Physics::wireForceCoeff = wireForceCoeff;
}

void Physics::setBarriersRestitutionCoeff(float barriersRestitutionCoeff) {
    Physics::barriersRestitutionCoeff = barriersRestitutionCoeff;
}

void Physics::setLinearDrag(float linearDrag) {
    Physics::linearDrag = linearDrag;
}

void Physics::setNoiseModulus(float noiseModulus) {
    Physics::noiseModulus = noiseModulus;
}

void Physics::setNoiseSpeedCoeff(float noiseSpeedCoeff) {
    Physics::noiseSpeedCoeff = noiseSpeedCoeff;
}

bool Physics::isEnableBarrierCollisions() const {
    return enableBarrierCollisions;
}

void Physics::setEnableBarrierCollisions(bool enableBarrierCollisions) {
    Physics::enableBarrierCollisions = enableBarrierCollisions;
}

float Physics::getIoForceMultiplier() const {
    return ioForceMultiplier;
}

void Physics::setIoForceMultiplier(float ioForceMultiplier) {
    Physics::ioForceMultiplier = ioForceMultiplier;
}

bool Physics::isEnableRegionCollisions() const {
    return enableRegionCollisions;
}

void Physics::setEnableRegionCollisions(bool enableRegionCollisions) {
    Physics::enableRegionCollisions = enableRegionCollisions;
}

float Physics::getSeparationCoeff() const {
    return separationCoeff;
}

void Physics::setFIXED_STEP_TIME(float FIXED_STEP_TIME) {
    Physics::FIXED_STEP_TIME = FIXED_STEP_TIME;
}

float Physics::getFIXED_STEP_TIME() const {
    return FIXED_STEP_TIME;
}

