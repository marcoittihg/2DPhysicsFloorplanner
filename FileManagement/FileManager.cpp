//
// Created by Marco on 28/12/17.
//


#include "FileManager.h"

Problem* FileManager::readProblem(std::string fileName) {
    //Open the file
    std::ifstream inFile(fileName, std::ios_base::in);

    if(!inFile.is_open())
        throw std::invalid_argument("Unable to open the file");

    Problem* problem = new Problem(&inFile);
    inFile.close();

    return problem;
}