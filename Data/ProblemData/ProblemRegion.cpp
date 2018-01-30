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
    return CLBNum;
}

int ProblemRegion::getBRAMNum() const {
    return BRAMNum;
}

int ProblemRegion::getDSPNum() const {
    return DSPNum;
}

int ProblemRegion::getIONum() const {
    return IONum;
}

const RegionIOData ProblemRegion::getRegionIO(int i) const {
    return regionIO[i];
}

ProblemRegion::~ProblemRegion() {
    delete [] regionIO;
}

ProblemRegion::ProblemRegion(std::ifstream *pIfstream) {
    char regType;
    *pIfstream >> regType;

    if(regType == RegionType::PR){
        type = RegionType ::PR;
    }else if(regType == RegionType::S){
        type = RegionType::S;
    }else{
        throw std::invalid_argument("Wrong region type in problem regions");
    }

    *pIfstream >> CLBNum;
    *pIfstream >> BRAMNum;
    *pIfstream >> DSPNum;

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

void ProblemRegion::setRegionIO(RegionIOData *regionIO) {
    ProblemRegion::regionIO = regionIO;
}

RegionIOData *ProblemRegion::getRegionIO() const {
    return regionIO;
}
