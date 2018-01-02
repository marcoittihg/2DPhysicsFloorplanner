//
// Created by Marco on 28/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_FILEMANAGER_H
#define BUBBLEREGIONSFLOORPLANNER_FILEMANAGER_H



#include <iostream>
#include <vector>
#include "../Data/ProblemData/Problem.h"
#include "../Solving/FeasiblePlacement.h"

class FileManager {

    FileManager() = default;

public:
    static FileManager& getINSTANCE(){
        static FileManager fileManager;
        return fileManager;
    }

    /**
     * @param fileName The path of the file to read
     * @return The instance of the problem generated from the file
     */
    Problem* readProblem(std::string fileName);

    void writeFeasiblePlacementToFile(std::vector<std::vector<FeasiblePlacement>> fp,Problem* problem);

    void readFeasiblePlacementToFile(std::string filePath, std::vector<std::vector<FeasiblePlacement>>* fp);


    //void writeSolutionToFile(Problem *pProblem, SolutionState *pState, std::vector<std::vector<FeasiblePlacement>> *pVector);
};



#endif //BUBBLEREGIONSFLOORPLANNER_FILEMANAGER_H
