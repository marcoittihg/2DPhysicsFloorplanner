//
// Created by Marco on 28/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_PHYSICS_H
#define BUBBLEREGIONSFLOORPLANNER_PHYSICS_H

#include <vector>
#include "../PhysicsRegion.h"
#include "../../Data/FPGAData/Board.h"
#include "Rigidbody.h";
#include "../PhysicsRegion.h"

typedef class Rigidbody;
typedef class PhysicsRegion;

/** Physics simulate the movement of the regions and applies forces
 */
class Physics {

public:
    /** Step time between each step of the physics
     */
    float FIXED_STEP_TIME;

    static Physics& getINSTANCE(){
        static Physics physics;
        return physics;
    }

private:

    static float constexpr MAX_STEP_TIME = 0.1;


    Physics() = default;

    /** Rigidbody that are simulated
     */
    Rigidbody* rigidbodies;

    /** Regions that are simulated
     */
    PhysicsRegion* physicsRegions;

    /** Number of regions for the physics
     */
    unsigned short regionNum;

    /** The board in which the
     *  regions are contined
     */
    Board* board;

    /** Coefficient for separation forces
     */
    float separationCoeff = 0;

    /** Coefficient that mulltiply the actraction
     * force to the prefered placement for a regions
     */
    float preferedAnchorCoeff = 0;

    float ioForceMultiplier = 1;

    /** Coefficient that multiply the actraction force
     *  to the closest feasible placement
     */
    float closestAnchorCoeff = 0;

    /** Multiply the force of interconnections and IOs
     */
    float wireForceCoeff = 1;

    /** Cofficient that multiplies the velocity of the region when hit a barrier
     */
    float barriersRestitutionCoeff = 0.8;

    /** Linear drag for step for each region
     */
    float linearDrag = 0.1;

    /** Maximum modulus of the noise
     */
    float noiseModulus;

    /** Speed change of the noise
     */
    float noiseSpeedCoeff;

    /** Noise force
     */
    Vector2 noise;

    /** Direction if increment of the noise
     */
    Vector2 noiseStepDir;

    bool enableBarrierCollisions;

    bool enableRegionCollisions;
public:

    /** Called at the start of the simulation
     */
    void onStart();

    /** Execute a step by calling the before physics step for each region
     *  and after that update the positions
     */
    void doStep();


    /**
     * Adds a region to the physics simulation
     */
    PhysicsRegion* addRegion();

    PhysicsRegion *getPhysicsRegions() const;

    unsigned short getRegionNum() const;

    Board *getBoard() const;

    void setBoard(Board *board);

    float getPreferedAnchorCoeff() const;

    float getClosestAnchorCoeff() const;

    float getWireForceCoeff() const;

    void setSeparationCoeff(float separationCoeff);

    void setPreferedAnchorCoeff(float preferedAnchorCoeff);

    void setClosestAnchorCoeff(float closestAnchorCoeff);

    void setWireForceCoeff(float wireForceCoeff);

    void setBarriersRestitutionCoeff(float barriersRestitutionCoeff);

    void setLinearDrag(float linearDrag);

    void setNoiseModulus(float noiseModulus);

    void setNoiseSpeedCoeff(float noiseSpeedCoeff);

    bool isEnableBarrierCollisions() const;

    void setEnableBarrierCollisions(bool enableBarrierCollisions);

    float getIoForceMultiplier() const;

    void setIoForceMultiplier(float ioForceMultiplier);

    bool isEnableRegionCollisions() const;

    void setEnableRegionCollisions(bool enableRegionCollisions);

    float getSeparationCoeff() const;

    void setFIXED_STEP_TIME(float FIXED_STEP_TIME);

    float getFIXED_STEP_TIME() const;
};


#endif //BUBBLEREGIONSFLOORPLANNER_PHYSICS_H
