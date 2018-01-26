//
// Created by Marco on 28/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_PHISICSREGION_H
#define BUBBLEREGIONSFLOORPLANNER_PHISICSREGION_H

#include <vector>
#include "../BaseUtils/Vector2.h"
#include "Phyisics/Rigidbody.h"
#include "FeasiblePlacement.h"
#include "PhysicsRegionState.h"
#include "PhysicsRegionType.h"
#include "../Data/ProblemData/RegionIOData.h"

typedef class Rigidbody;

/** Physics entity of the region
 */
class PhysicsRegion {
    /** Index identifier of the region
     */
    unsigned char regionIndex;

    /** Rigidbody of the region
     */
    Rigidbody* rb;

    /**
     */
    PhysicsRegionState regionState;

    /** Type of the region
     */
    PhysicsRegionType regionType;

    /** Array containing all the feasible placements for the region
     */
    FeasiblePlacement* feasiblePlacements;

    /** Number of feasible placements for the region
     */
    unsigned short placementNum;

    /** The prefered feasible placement index
     */
    unsigned short preferdPlacementIndex;

    float anchorForceMultiplier = 1;


    Vector2 preferedAnchorPoint = Vector2(0,0);

    Vector2 wireStabilityPosition;

    /** Regions that is interconnected with
     */
    std::vector<PhysicsRegion*> interconnectedRegions;

    std::vector<int> interconnectedRegionsWeights;

    /** Number of IO
     */
    char IONum;

    /** IO datas for the region
     */
    RegionIOData* regionIO;

public:

    PhysicsRegion();

    void fixedPhysicsStep();

    void onPhysicsStart();

    unsigned char getRegionIndex() const;

    void setRegionIndex(unsigned char regionIndex);

    Rigidbody *getRb() const;

    void setRb(Rigidbody *rb);

    FeasiblePlacement *getFeasiblePlacements() const;

    void setFeasiblePlacements(FeasiblePlacement *feasiblePlacements);

    unsigned short getPreferdPlacementIndex() const;

    void setPreferdPlacementIndex(unsigned short preferdPlacementIndex);

    const std::vector<PhysicsRegion *> &getInterconnectedRegions() const;

    void setInterconnectedRegions(const std::vector<PhysicsRegion *> &interconnectedRegions);

    void addInterconnectedRegion(PhysicsRegion* newRegion, int weight);

    PhysicsRegionState getRegionState() const;

    void setRegionState(PhysicsRegionState regionState);

    Vector2 getPreferedAnchorPoint();


    void setPreferedAnchorPoint(const Vector2 &preferedAnchorPoint);


    PhysicsRegionType getRegionType() const;

    void setRegionType(PhysicsRegionType regionType);

    char getIONum() const;

    void setIONum(char IONum);

    RegionIOData *getRegionIO() const;

    void setRegionIO(RegionIOData *regionIO);

    void resetPositionAndShape();

    void evaluatePlacementAndShape(bool isStart);

    void savePositionAsWireStability();

    const Vector2 &getWireStabilityPosition() const;


    unsigned short getPlacementNum() const;

    void setPlacementNum(unsigned short placementNum);

    unsigned int evaluatePlacement(FeasiblePlacement fp, bool isLastStep);
};


#endif //BUBBLEREGIONSFLOORPLANNER_PHISICSREGION_H
