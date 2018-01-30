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

Board::Board(std::ifstream *pIfstream) {
    int num;

	*pIfstream >> num;
	dimension.set_y(num);
	*pIfstream >> num;
	dimension.set_x(num);

	int th;
	*pIfstream >> th;

	tileHeight = th;

	std::map<Block, int> emptyBorder;
	emptyBorder[Block::BRAM_BLOCK] = 0;
	emptyBorder[Block::CLB_BLOCK] = 0;
	emptyBorder[Block::DSP_BLOCK] = 0;
	emptyBorder[Block::FORBIDDEN_BLOCK] = 0;
	emptyBorder[Block::NULL_BLOCK] = 0;

	this->_resourcesFromOrigin = new std::map<Block, int>*[this->dimension.get_y() + 1];

	// Populate resource counter for each square originating from (0, 0) 
	for (int y = 0; y <= dimension.get_y(); y++) {
		this->_resourcesFromOrigin[y] = new std::map<Block, int>[this->dimension.get_x() + 1];

		for (int x = 0; x <= dimension.get_x(); x++) {

			if (x == 0 || y == 0) {
				this->_resourcesFromOrigin[y][x] = emptyBorder;
				continue;
			}

			char blk;
			*pIfstream >> blk;

			if (y == 1) {
				this->_resourcesFromOrigin[y][x] = this->_resourcesFromOrigin[y][x - 1];
				this->_resourcesFromOrigin[y][x][(Block)blk]++;
			}
			else {
				if (x == 1) {
					this->_resourcesFromOrigin[y][x] = this->_resourcesFromOrigin[y - 1][x];
					this->_resourcesFromOrigin[y][x][(Block)blk]++;
				}
				else {
					this->_resourcesFromOrigin[y][x] = this->_resourcesFromOrigin[y - 1][x];
					auto left = this->_resourcesFromOrigin[y][x - 1];
					auto upleft = this->_resourcesFromOrigin[y - 1][x - 1];

					for (auto it = left.begin(); it != left.end(); ++it)
						this->_resourcesFromOrigin[y][x][it->first] += it->second - upleft.at(it->first);

					this->_resourcesFromOrigin[y][x][(Block)blk]++;
				}
			}
		}
	}
}

Board::~Board() {
	for (int y = 0; y < dimension.get_y(); y++) {
		delete[] this->_resourcesFromOrigin[y];
	}
	delete[] this->_resourcesFromOrigin;
}

const std::map<Block, int> Board::getResourcesFor(Point2D start, Point2D dimension) const {

	int startX = start.get_x(), startY = start.get_y();
	int endX = startX + dimension.get_x(), endY = startY + dimension.get_y();

	std::map<Block, int> result(this->_resourcesFromOrigin[endY][endX]);
	auto upleft = &(this->_resourcesFromOrigin[startY][startX]);
	auto up = &(this->_resourcesFromOrigin[startY][endX]);
	auto left = &(this->_resourcesFromOrigin[endY][startX]);

	for (auto it = result.begin(); it != result.end(); ++it)
		it->second += upleft->at(it->first) - left->at(it->first) - up->at(it->first);

	return result;
}