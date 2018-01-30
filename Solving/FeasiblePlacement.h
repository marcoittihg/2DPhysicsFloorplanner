//
// Created by Marco on 28/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_FEASIBLEPLACEMENT_H
#define BUBBLEREGIONSFLOORPLANNER_FEASIBLEPLACEMENT_H


#include "../BaseUtils/Point2D.h"
#include "../Data/ProblemData/RegionType.h"
#include "FeasiblePlacementState.h"
#include "Resources.h"
#include "../Data/FPGAData/Board.h"

class FeasiblePlacement {
    /**Starting position of the placement
     */
    Point2D startPosition;

    /**Dimension of the placement
     */
    Point2D dimension;

    /** Type of the placed region
     */
    RegionType regionType;

    /** State of the feasible placement
     * Contains if the placement is already taken, blocked or available
     */
    FeasiblePlacementState feasiblePlacementState;

    /** Precomputed area cost of the placement
     */
    unsigned int areaCost;

    /** Computed score loss for the feasible placement
     */
    unsigned int scoreLoss;

    /** Resorces occupied by the placement
     */
    Resources resources;

public:

	FeasiblePlacement();
	FeasiblePlacement(Point2D start, Point2D dimension, RegionType rType, int aCost);

    const Point2D &getStartPosition() const;

    void setStartPosition(const Point2D &startPosition);

    const Point2D &getDimension() const;

    void setDimension(const Point2D &dimension);

    RegionType getRegionType() const;

    void setRegionType(RegionType regionType);

    FeasiblePlacementState getFeasiblePlacementState() const;

    void setFeasiblePlacementState(FeasiblePlacementState feasiblePlacementState);

    unsigned int getAreaCost() const;

    void setAreaCost(unsigned int areaCost);

    unsigned int getScoreLoss() const;

    void setScoreLoss(unsigned int scoreLoss);

    bool checkContains(FeasiblePlacement *feasiblePlacement) const;

    void calculateResources(Board* board);

    const Resources &getResources() const;

	int getArea() const;

    /**
     * Check if two placements collides
     * @return The result of the check
     */
    inline static bool checkCollision(FeasiblePlacement* fp1, FeasiblePlacement* fp2 , char tileHeight){

        Point2D startPos1 = fp1->startPosition;
        Point2D startPos2 = fp2->startPosition;

        Point2D dim1 = fp1->dimension;
        Point2D dim2 = fp2->dimension;


        if(fp1->regionType == RegionType::PR){
            Point2D oldPos = startPos1;

            startPos1.set_y((startPos1.get_y() / tileHeight)* tileHeight);
            dim1.set_y(dim1.get_y() + (oldPos.get_y() - startPos1.get_y()));

            if(dim1.get_y() % tileHeight != 0){
                dim1.set_y(((dim1.get_y() / tileHeight) + (char) 1) * tileHeight);
            }
        }

        if(fp2->regionType == RegionType::PR){
            Point2D oldPos = startPos2;

            startPos2.set_y((startPos2.get_y() / tileHeight)* tileHeight);
            dim2.set_y(dim2.get_y() + (oldPos.get_y() - startPos2.get_y()));

            if(dim2.get_y() % tileHeight != 0){
                dim2.set_y(((dim2.get_y() / tileHeight) + (char) 1) * tileHeight);
            }
        }

        return !(
                startPos1.get_x() + dim1.get_x() <= startPos2.get_x() ||
                startPos2.get_x() + dim2.get_x() <= startPos1.get_x() ||
                startPos1.get_y() + dim1.get_y() <= startPos2.get_y() ||
                startPos2.get_y() + dim2.get_y() <= startPos1.get_y()
        );
    }
};


#endif //BUBBLEREGIONSFLOORPLANNER_FEASIBLEPLACEMENT_H
