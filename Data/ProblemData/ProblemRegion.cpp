//
// Created by Marco on 28/12/17.
//


#include "ProblemRegion.h"
#include "RegionIOData.h"
#include "RegionType.h"

RegionType ProblemRegion::getType() const {
    return type;
}

int ProblemRegion::getCLBNum() const {
    return this->_resources.at(Block::CLB_BLOCK);
}

int ProblemRegion::getBRAMNum() const {
    return this->_resources.at(Block::BRAM_BLOCK);
}

int ProblemRegion::getDSPNum() const {
    return this->_resources.at(Block::DSP_BLOCK);
}

int ProblemRegion::getIONum() const {
    return IONum;
}

int ProblemRegion::getID() const {
	return this->_regionID;
}

const RegionIOData ProblemRegion::getRegionIO(int i) const {
    return regionIO[i];
}

ProblemRegion::ProblemRegion(std::ifstream *pIfstream, int regionID) {
	this->_regionID = regionID;

    char regType;
    *pIfstream >> regType;

    if(regType == RegionType::PR){
        type = RegionType ::PR;
    }else if(regType == RegionType::S){
        type = RegionType::S;
    }else{
        throw std::invalid_argument("Wrong region type in problem regions");
    }

	*pIfstream >> this->_resources[Block::CLB_BLOCK];
	*pIfstream >> this->_resources[Block::BRAM_BLOCK];
	*pIfstream >> this->_resources[Block::DSP_BLOCK];

    *pIfstream >> IONum;

    unsigned short int portColumn;
    unsigned short int portRow;
    unsigned short int numWires;

    regionIO = new RegionIOData[IONum];

    for (int i = 0; i < IONum; ++i) {
        *pIfstream >> portColumn;
        regionIO[i].setPortColumn(portColumn);
        *pIfstream >> portRow;
        regionIO[i].setPortRow(portRow);
        *pIfstream >> numWires;
        regionIO[i].setNumWires(numWires);
    }
}

ProblemRegion::~ProblemRegion() {
	delete[] regionIO;
}

void ProblemRegion::setRegionIO(RegionIOData *regionIO) {
    ProblemRegion::regionIO = regionIO;
}

RegionIOData *ProblemRegion::getRegionIO() const {
    return regionIO;
}

bool ProblemRegion::isContained(const std::map<Block, int>* availableResources) const {

	// Check for null pointer for given resource map
	if (availableResources == NULL)
		throw new std::exception("Given available resources map pointer cannot be null.");

	// If given resources map contains forbidden blocks return false
	if (availableResources->find(Block::FORBIDDEN_BLOCK) != availableResources->end()
		&& availableResources->at(Block::FORBIDDEN_BLOCK) > 0)
		return false;

	// Check that for each resource there is enough in the availableResources map
	for (auto it = this->_resources.begin(); it != this->_resources.end(); ++it) {
		if (availableResources->at(it->first) < it->second) return false;
	}

	return true;
}