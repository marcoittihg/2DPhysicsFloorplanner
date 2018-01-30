//
// Created by Marco on 28/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_BOARD_H
#define BUBBLEREGIONSFLOORPLANNER_BOARD_H


#include <fstream>
#include "../../BaseUtils/Point2D.h"
#include "Block.h"
#include <map>

/** Datas about a board of a specific problem
 */
class Board{
    /** Width and height of the board
     */
    Point2D dimension;

    /** Height of the tiles of the board
     */
    char tileHeight;

	/**
	*	Precalculated resources map from origin to given point
	*	as [x][y] coordinates on the board
	*/
	std::map<Block, int>** _resourcesFromOrigin;
public:
    virtual ~Board();

    Board(std::ifstream *pIfstream);

    const Point2D &getDimension() const;

    char getTileHeight() const;

	const std::map<Block, int> getResourcesFor(Point2D start, Point2D dimension) const;
};
#endif //BUBBLEREGIONSFLOORPLANNER_BOARD_H
