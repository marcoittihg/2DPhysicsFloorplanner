//
// Created by Marco on 28/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_PROBLEMREGION_H
#define BUBBLEREGIONSFLOORPLANNER_PROBLEMREGION_H

#include <fstream>
#include <map>
#include "../FPGAData/Block.h"
#include "RegionType.h"
#include "RegionIOData.h"

class ProblemRegion {
    /** Type of the region
     */
    RegionType type;

    /** Number of IO
     */
    int IONum;

    /** IO datas for the region
     */
    RegionIOData* regionIO;

	/** Number of resources (CLBs, DSPs, BRAMs)
	*/
	std::map<Block, int> _resources;

	int _regionID;

public:

    ProblemRegion(std::ifstream *pIfstream, int regionID);
	virtual ~ProblemRegion();

    RegionType getType() const;

    int getCLBNum() const;

    int getBRAMNum() const;

    int getDSPNum() const;

    int getIONum() const;

	int getID() const;

	bool isContained(const std::map<Block, int>* availableResources) const;

    const RegionIOData getRegionIO(int) const;

    void setRegionIO(RegionIOData *regionIO);

    RegionIOData *getRegionIO() const;
};


#endif //BUBBLEREGIONSFLOORPLANNER_PROBLEMREGION_H
