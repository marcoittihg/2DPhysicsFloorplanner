//
// Created by Marco on 28/12/17.
//


#include "FileManager.h"
#include "../Data/ProblemData/RegionType.h"
#include <fstream>

Problem* FileManager::readProblem(std::string fileName) {
    //Open the file
    std::ifstream inFile(fileName, std::ios_base::in);

    if(!inFile.is_open())
        throw std::invalid_argument("Unable to open the file");


    Problem* problem = new Problem(&inFile);
    inFile.close();

    return problem;
}

void FileManager::writeFeasiblePlacementToFile(std::vector<std::vector<FeasiblePlacement>> fp, Problem* problem) {
    std::string problemID = std::to_string(problem->getID());
    std::string newFileName = problemID+"Regions.txt";
    std::ofstream fostream(newFileName ,std::ios_base::out);

    fostream <<fp.size()<<std::endl;

    //Write feasible placements number per region
    for (int i = 0; i < fp.size(); ++i) {
        fostream <<fp.at(i).size()<<" ";
    }

    fostream << std::endl;

    //Write all the regions
    for (int i = 0; i < fp.size(); ++i) {
        for (int j = 0; j < fp.at(i).size(); ++j) {
            fostream << fp.at(i).at(j).getRegionType() <<" "
                     << +fp.at(i).at(j).getStartPosition().get_x() <<" "<< +fp.at(i).at(j).getStartPosition().get_y()<<" "
                     << +fp.at(i).at(j).getDimension().get_x() <<" "<< +fp.at(i).at(j).getDimension().get_y()
                     << std::endl;
        }
    }

    fostream.flush();
    fostream.close();
}


void FileManager::readFeasiblePlacementToFile(std::string filePath, std::vector<std::vector<FeasiblePlacement>>* fp) {
    std::ifstream fistream(filePath ,std::ios_base::in);

    int numRegions;
    fistream >> numRegions;

    fp->resize(numRegions);

    for (int i = 0; i < numRegions; ++i) {
        int size;
        fistream >> size;

        fp->at(i).resize(size);
    }

    Point2D startPos, dimension;
    for (int i = 0; i < numRegions; ++i) {
        for (int j = 0; j < fp->at(i).size(); ++j) {
            int rt;
            int startX,startY,dimX,dimY;
            fistream >> rt >> startX >> startY >> dimX >> dimY;
            fp->at(i).at(j).setRegionType( static_cast<RegionType>(rt) );

            startPos.set_x(startX);
            startPos.set_y(startY);

            dimension.set_x(dimX);
            dimension.set_y(dimY);

            fp->at(i).at(j).setStartPosition(startPos);
            fp->at(i).at(j).setDimension(dimension);
        }
    }
}