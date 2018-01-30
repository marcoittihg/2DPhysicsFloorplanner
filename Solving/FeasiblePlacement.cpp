//
// Created by Marco on 28/12/17.
//

#include "FeasiblePlacement.h"

FeasiblePlacement::FeasiblePlacement() {

}

FeasiblePlacement::FeasiblePlacement(Point2D start, Point2D dimension, RegionType rType, int aCost) {
	this->startPosition = start;
	this->dimension = dimension;
	this->regionType = rType;
	this->areaCost = aCost;
}

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
    this->areaCost = areaCost;
}

unsigned int FeasiblePlacement::getScoreLoss() const {
    return scoreLoss;
}

void FeasiblePlacement::setScoreLoss(unsigned int scoreLoss) {
    this->scoreLoss = scoreLoss;
}


bool FeasiblePlacement::checkContains(FeasiblePlacement * feasiblePlacement)const {
    return this->startPosition.get_x() <= feasiblePlacement->startPosition.get_x() &&
           this->startPosition.get_y() <= feasiblePlacement->startPosition.get_y() &&
           this->startPosition.get_x() + this->dimension.get_x() >= feasiblePlacement->startPosition.get_x() + feasiblePlacement->dimension.get_x() &&
           this->startPosition.get_y() + this->dimension.get_y() >= feasiblePlacement->startPosition.get_y() + feasiblePlacement->dimension.get_y();
}

int FeasiblePlacement::getArea() const {
	return this->dimension.get_x() * this->dimension.get_y();
}

void FeasiblePlacement::calculateResources(Board* board) {
	resources.BRAM = resources.CLB = resources.DSP = 0;

	const std::map<Block, int> resources = board->getResourcesFor(this->startPosition, this->dimension);

	this->resources.CLB = resources.at(Block::CLB_BLOCK);
	this->resources.DSP = resources.at(Block::DSP_BLOCK);
	this->resources.BRAM = resources.at(Block::BRAM_BLOCK);
}

const Resources &FeasiblePlacement::getResources() const {
    return this->resources;
}
