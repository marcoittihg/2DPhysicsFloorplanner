//
// Created by Marco on 28/12/17.
//


#include "RegionIOData.h"

unsigned short int RegionIOData::getPortColumn() const {
    return portColumn;
}

unsigned short int RegionIOData::getPortRow() const {
    return portRow;
}

unsigned short int RegionIOData::getNumWires() const {
    return numWires;
}

RegionIOData::RegionIOData() {}

void RegionIOData::setPortColumn(unsigned short portColumn) {
    RegionIOData::portColumn = portColumn;
}

void RegionIOData::setPortRow(unsigned short portRow) {
    RegionIOData::portRow = portRow;
}

void RegionIOData::setNumWires(unsigned short numWires) {
    RegionIOData::numWires = numWires;
}
