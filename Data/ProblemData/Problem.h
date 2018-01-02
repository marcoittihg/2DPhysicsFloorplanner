//
// Created by Marco on 28/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_PROBLEM_H
#define BUBBLEREGIONSFLOORPLANNER_PROBLEM_H



#include <fstream>
#include "../FPGAData/Board.h"
#include "ProblemRegion.h"

/** Contains the data of a requested problem
 */
class Problem {
    /** ID of the problem
     */
    int ID;

    /** Maximum score for the problem
     */
    int maxScore;

    /** Cost weight for the area
     */
    int areaCost;

    /** Cost weight of the connections
     */
    int wireCost;

    /** Cost of each CLB block
     */
    int CLBCost;

    /** Cost of each BRAM block
     */
    int BRAMCost;

    /** Cost of each DSP block
     */
    int DSPCost;

    /** Data containing the components of the board
     */
    Board *board;

    /** Array containing the left valid IDs
     */
    bool *leftValidIDs;

    /** Array containing the right valid IDs
     */
    bool *rightValidIDs;

    /** Nuber of requested region for the problem
     */
    int numRegions;

    /** Data of the requested regions
     */
    ProblemRegion **floorplanProblemRegion;

    /** Contains the number of interconnections among the areas of the problem
     */
    int **interconnectionsMatrix;

public:
    virtual ~Problem();

public:
    Problem(std::ifstream *inFile);

    int getID() const;

    int getMaxScore() const;

    int getAreaCost() const;

    int getWireCost() const;

    int getCLBCost() const;

    int getBRAMCost() const;

    int getDSPCost() const;

    Board *getBoard() const;

    const bool getLeftValidIDs(int) const;

    const bool getRightValidIDs(int) const;

    int getNumRegions() const;

    const ProblemRegion *getFloorplanProblemRegion(int) const;

    int getInterconnectionsMatrix(int, int) const;

    int **getInterconnectionsMatrix() const;
};

#endif //BUBBLEREGIONSFLOORPLANNER_PROBLEM_H
