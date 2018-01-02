//
// Created by Marco on 28/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_PROBLEMREGION_H
#define BUBBLEREGIONSFLOORPLANNER_PROBLEMREGION_H


#include <fstream>
#include "RegionType.h"
#include "RegionIOData.h"

class ProblemRegion {
    /** Type of the region
     */
    RegionType type;

    /** Number of requested CLB
     */
    int CLBNum;

    /** Number of requested BRAM
     */
    int BRAMNum;

    /** Number of requested DSP
     */
    int DSPNum;

    /** Number of IO
     */
    int IONum;

    /** IO datas for the region
     */
    RegionIOData* regionIO;
public:
    virtual ~ProblemRegion();

    ProblemRegion(std::__1::basic_ifstream<char> *pIfstream);

public:
    RegionType getType() const;

    int getCLBNum() const;

    int getBRAMNum() const;

    int getDSPNum() const;

    int getIONum() const;

    const RegionIOData getRegionIO(int) const;

    void setRegionIO(RegionIOData *regionIO);

    RegionIOData *getRegionIO() const;
};


#endif //BUBBLEREGIONSFLOORPLANNER_PROBLEMREGION_H
