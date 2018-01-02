//
// Created by Marco on 28/12/17.
//

#include "Problem.h"
#include "../FPGAData/Board.h"
#include <fstream>

int Problem::getID() const {
    return ID;
}

int Problem::getMaxScore() const {
    return maxScore;
}

int Problem::getAreaCost() const {
    return areaCost;
}

int Problem::getWireCost() const {
    return wireCost;
}

int Problem::getCLBCost() const {
    return CLBCost;
}

int Problem::getBRAMCost() const {
    return BRAMCost;
}

int Problem::getDSPCost() const {
    return DSPCost;
}

Board * Problem::getBoard() const {
    return board;
}

const bool Problem::getLeftValidIDs(int i) const {
    return leftValidIDs[i];
}

const bool Problem::getRightValidIDs(int i) const {
    return rightValidIDs[i];
}

int Problem::getNumRegions() const {
    return numRegions;
}

const ProblemRegion *Problem::getFloorplanProblemRegion(int i) const {
    return floorplanProblemRegion[i];
}

int Problem::getInterconnectionsMatrix(int i, int j) const {
    return interconnectionsMatrix[i][j];
}

Problem::~Problem() {
    delete [] leftValidIDs;
    delete [] rightValidIDs;

    for (int i = 0; i < numRegions; i++) {
        delete []interconnectionsMatrix[i];
    }
}

Problem::Problem(std::ifstream* inFile) {
    *inFile >> ID;
    *inFile >> maxScore;
    *inFile >> areaCost;
    *inFile >> wireCost;
    *inFile >> CLBCost;
    *inFile >> BRAMCost;
    *inFile >> DSPCost;

    board = new Board(inFile);

    leftValidIDs = new bool[board->getDimension().get_y()];
    for (int i = 0; i < board->getDimension().get_y(); i++) {
        *inFile >> leftValidIDs[i];
    }

    rightValidIDs = new bool[board->getDimension().get_y()];
    for (int i = 0; i < board->getDimension().get_y(); i++) {
        *inFile >> rightValidIDs[i];
    }

    *inFile >> numRegions;
    floorplanProblemRegion = new ProblemRegion*[numRegions];
    for (int k = 0; k < numRegions; ++k) {
        floorplanProblemRegion[k] = new ProblemRegion(inFile);
    }

    interconnectionsMatrix = new int*[numRegions];
    for (int i = 0; i < numRegions; ++i) {
        interconnectionsMatrix[i] = new int[numRegions];
        for (int j = 0; j < numRegions; ++j) {
            *inFile >> interconnectionsMatrix[i][j];
        }
    }
}

int **Problem::getInterconnectionsMatrix() const {
    return interconnectionsMatrix;
}
