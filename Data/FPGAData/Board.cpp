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

Block Board::getBlockMatrix(char i, char j) const {
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