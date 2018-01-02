//
// Created by Marco on 28/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_REGIONIODATA_H
#define BUBBLEREGIONSFLOORPLANNER_REGIONIODATA_H


#include <fstream>

/** Data of an IO for a ProblemRegion
 */
class RegionIOData{
    unsigned short int portColumn;
    unsigned short int portRow;

    /** Number of wires to connect to the IO
     */
    unsigned short int numWires;
public:
    RegionIOData();

public:
    unsigned short int getPortColumn() const;

    unsigned short int getPortRow() const;

    void setPortColumn(unsigned short portColumn);

    void setPortRow(unsigned short portRow);

    void setNumWires(unsigned short numWires);

    unsigned short int getNumWires() const;

};

#endif //BUBBLEREGIONSFLOORPLANNER_REGIONIODATA_H
