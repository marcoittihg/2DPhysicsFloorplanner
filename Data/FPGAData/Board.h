//
// Created by Marco on 28/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_BOARD_H
#define BUBBLEREGIONSFLOORPLANNER_BOARD_H


#include <fstream>
#include "../../BaseUtils/Point2D.h"
#include "Block.h"
#include "../../Solving/Resources.h"

/** Datas about a board of a specific problem
 */
class Board{
    /** Width and height of the board
     */
    Point2D dimension;

    /** Height of the tiles of the board
     */
    char tileHeight;

    /** Interconnection matrix
     *  blockMatrix[i][j] - return the block at position[i][j]
     */
    Block ** blockMatrix;

    /** Matrix containig the cumulative resource matrix for the board
     */
    Resources** cumulativeResources = nullptr;

public:
    virtual ~Board();

    Board(std::__1::basic_ifstream<char> *pIfstream);

    const Point2D &getDimension() const;

    char getTileHeight() const;

    /**
     * @return the value of the block matrix at position [i][j]
     */
    Block getBlockMatrix(unsigned char, unsigned char) const;

    void computeCumulativeResourceMatrix();

    Resources getBoardResources(Point2D start, Point2D end);
};
#endif //BUBBLEREGIONSFLOORPLANNER_BOARD_H
