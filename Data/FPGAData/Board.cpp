//
// Created by Marco on 28/12/17.
//


#include "Board.h"

const Point2D &Board::getDimension() const {
    return dimension;
}

char Board::getTileHeight() const {
    return tileHeight;
}

Block Board::getBlockMatrix(unsigned char i, unsigned char j) const {
    return blockMatrix[i][j];
}

Board::Board(std::ifstream *pIfstream) {
    int num;

    *pIfstream >> num;
    dimension.set_x(num);
    *pIfstream >> num;
    dimension.set_y(num);

    int th;
    *pIfstream >> th;

    tileHeight = th;

    blockMatrix = new Block*[dimension.get_x()];
    for (int i = 0; i < dimension.get_x(); i++) {
        blockMatrix[i] = new Block[dimension.get_y()];

        for (int j = 0; j < dimension.get_y(); ++j) {
            char blk;
            *pIfstream >> blk;

            switch(blk){
                case Block ::CLB_BLOCK:
                    blockMatrix[i][j] = Block :: CLB_BLOCK;
                    break;
                case Block ::BRAM_BLOCK:
                    blockMatrix[i][j] = Block :: BRAM_BLOCK;
                    break;
                case Block ::DSP_BLOCK:
                    blockMatrix[i][j] = Block :: DSP_BLOCK;
                    break;
                case Block ::FORBIDDEN_BLOCK:
                    blockMatrix[i][j] = Block :: FORBIDDEN_BLOCK;
                    break;
                case Block ::NULL_BLOCK:
                    blockMatrix[i][j] = Block :: NULL_BLOCK;
                    break;
                default:
                    throw std::invalid_argument("Error while reading blocks matrix");
            }
        }
    }
}

Board::~Board() {
    for (int i = 0; i < dimension.get_x(); i++) {
        delete [] blockMatrix[i];
    }
    delete [] blockMatrix;
}

void Board::computeCumulativeResourceMatrix() {
    //Allocate the matrix if not yet allocated
    if(cumulativeResources == nullptr) {
        cumulativeResources = new Resources *[dimension.get_y()];
        for (int j = 0; j < dimension.get_y(); ++j) {
            cumulativeResources[j] = new Resources[dimension.get_x()];
        }
    }

    //Compute matrix
    for (int i = 0; i < dimension.get_y(); ++i) {
        for (int j = 0; j < dimension.get_x(); ++j) {
            if(i==0 && j== 0) {
                cumulativeResources[0][0].addBlock(blockMatrix[0][0]);
                continue;
            }

            if(i == 0){
                cumulativeResources[i][j] = cumulativeResources[0][j-1];
                cumulativeResources[i][j].addBlock(blockMatrix[j][i]);
                continue;
            }

            if(j == 0){
                cumulativeResources[i][j] = cumulativeResources[i-1][j];
                cumulativeResources[i][j].addBlock(blockMatrix[j][i]);
                continue;
            }

            //i and j != 0
            cumulativeResources[i][j].add(cumulativeResources[i-1][j]);
            cumulativeResources[i][j].add(cumulativeResources[i][j-1]);
            cumulativeResources[i][j].sub(cumulativeResources[i-1][j-1]);
            cumulativeResources[i][j].addBlock(blockMatrix[j][i]);
        }
    }

}

Resources Board::getBoardResources(Point2D start, Point2D end) {
    Resources res;

    res.add(cumulativeResources[end.get_y()][end.get_x()]);

    if(start.get_x() != 0 && start.get_y() != 0) {
        res.add(cumulativeResources[start.get_y() - 1][start.get_x() - 1]);
    }

    if(start.get_x() != 0){
        res.sub(cumulativeResources[end.get_y()][start.get_x() - 1]);
    }
    if(start.get_y() != 0){
        res.sub(cumulativeResources[start.get_y() - 1][end.get_x()]);
    }

    return res;
}
