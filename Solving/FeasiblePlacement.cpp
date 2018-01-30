//
// Created by Marco on 28/12/17.
//

#include "FeasiblePlacement.h"

const Point2D &FeasiblePlacement::getStartPosition() const {
    return startPosition;
}

void FeasiblePlacement::setStartPosition(const Point2D &startPosition) {
    FeasiblePlacement::startPosition = startPosition;
}

const Point2D &FeasiblePlacement::getDimension() const {
    return dimension;
}

void FeasiblePlacement::setDimension(const Point2D &dimension) {
    FeasiblePlacement::dimension = dimension;
}

RegionType FeasiblePlacement::getRegionType() const {
    return regionType;
}

void FeasiblePlacement::setRegionType(RegionType regionType) {
    FeasiblePlacement::regionType = regionType;
}

FeasiblePlacementState FeasiblePlacement::getFeasiblePlacementState() const {
    return feasiblePlacementState;
}

void FeasiblePlacement::setFeasiblePlacementState(FeasiblePlacementState feasiblePlacementState) {
    FeasiblePlacement::feasiblePlacementState = feasiblePlacementState;
}

unsigned int FeasiblePlacement::getAreaCost() const {
    return areaCost;
}

void FeasiblePlacement::setAreaCost(unsigned int areaCost) {
    FeasiblePlacement::areaCost = areaCost;
}

unsigned int FeasiblePlacement::getScoreLoss() const {
    return scoreLoss;
}

void FeasiblePlacement::setScoreLoss(unsigned int scoreLoss) {
    FeasiblePlacement::scoreLoss = scoreLoss;
}


bool FeasiblePlacement::checkContains(FeasiblePlacement * feasiblePlacement)const {
    return this->startPosition.get_x() <= feasiblePlacement->startPosition.get_x() &&
           this->startPosition.get_y() <= feasiblePlacement->startPosition.get_y() &&
           this->startPosition.get_x() + this->dimension.get_x() >= feasiblePlacement->startPosition.get_x() + feasiblePlacement->dimension.get_x() &&
           this->startPosition.get_y() + this->dimension.get_y() >= feasiblePlacement->startPosition.get_y() + feasiblePlacement->dimension.get_y();
}

void FeasiblePlacement::calculateResources(Board* board) {
    resources.BRAM = resources.CLB = resources.DSP = 0;

    for (int i = startPosition.get_y(); i < startPosition.get_y() + dimension.get_y(); ++i) {
        for (int j = startPosition.get_x(); j < startPosition.get_x() + dimension.get_x(); ++j) {
            Block block = board->getBlockMatrix(i, j);

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
}

const Resources &FeasiblePlacement::getResources() const {
    return resources;
}
