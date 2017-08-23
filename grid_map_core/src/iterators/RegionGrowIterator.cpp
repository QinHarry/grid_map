/*
 * RegionGrowIterator.cpp
 *
 *  Created on: Aug 22 2017
 *      Author: Tobias Meier, tmeier@anybotics.com
 *   Institute: ANYbotics AG
 */

#include "grid_map_core/iterators/RegionGrowIterator.hpp";
#include "grid_map_core/GridMapMath.hpp";

//using namespace std;

namespace grid_map {


RegionGrowIterator::RegionGrowIterator(const GridMap& gridMap, const std::string& layer, const Position& start,
		const float lowerThreshold, const float upperThreshold)
	:data_(gridMap[layer])

{

	gridMap.getIndex(start, currentStartIndex_);
	mapLength_ = gridMap.getLength();
	currentIndex_= currentStartIndex_;
	lowerThreshold_ = lowerThreshold;
	upperThreshold_ = upperThreshold;
	referenceValue_ = gridMap.at(layer,currentStartIndex_);
	reverse_ = false;
	complete_ = false;

}

RegionGrowIterator& RegionGrowIterator::operator =(const RegionGrowIterator& other)
{
	mapLength_ = other.mapLength_;
	currentStartIndex_ = other.currentStartIndex_;
	currentIndex_= other.currentIndex_;
	lowerThreshold_ = other.lowerThreshold_;
	upperThreshold_ = other.upperThreshold_;
	referenceValue_ = other.referenceValue_;

	reverse_ = other.reverse_;
	complete_ = other.reverse_;

	return *this;
}

const Index& RegionGrowIterator::operator *() const
{
	return currentIndex_;
}

bool RegionGrowIterator::isPastEnd()
{
	return (complete_ && futureStartIndices_.empty());
}

RegionGrowIterator& RegionGrowIterator::operator ++()
{
	if (currentIndex_ == currentStartIndex_)
	{
		tempIndex_.x() = currentStartIndex_.x();
		tempIndex_.y() = currentStartIndex_.y() + 1;

		if(std::find(pastStartIndices_.begin(), pastStartIndices_.end(), tempIndex_) != pastStartIndices_.end() && tempIndex_.y() < mapLength_.y())
		{
		    futureStartIndices_.push_back(tempIndex_);
		}

		tempIndex_.y() -= 2;
		if(std::find(pastStartIndices_.begin(), pastStartIndices_.end(), tempIndex_) != pastStartIndices_.end() && tempIndex_.y() > 0)
				{
				    futureStartIndices_.push_back(tempIndex_);
				}

	}

	checkLine();




	return *this;
}

void RegionGrowIterator::checkLine()
{

	switch(isWithinThreshold(currentIndex_)) {

	    case true :

	    	if(!reverse_ && currentIndex_.x() < mapLength_.x())

	    		currentIndex_.x() += 1;
	    	else if (!reverse_ && currentIndex_.x() > 0)
	    		currentIndex_.x() -= 1;

	    	break;

	    case false :
	    	if(!reverse_)
	    	{
	    		// Go reverse from starting point
	    		currentIndex_.x()= currentStartIndex_.x() - 1;
	    		reverse_ = true;
	    	}
	    	else
	    	{
	    		// get starting points for next line
	    		// TODO: find further starting points, e..g. after a barrier.
	    		pastStartIndices_.push_back(currentStartIndex_);
	    		if (!futureStartIndices_.empty())
	    		{
	    			currentStartIndex_ = futureStartIndices_.back();
	    		}
	    		else
	    		{
	    			complete_ = true;
	    		}

	    		reverse_ = false;
	    	}
	    	break;
	}

}



bool RegionGrowIterator::isWithinThreshold(const Index index) const
{
	if (data_[index.x(), index.y()] >= *referenceValue_ - lowerThreshold_ && data_[index.x(), index.y()] <= *referenceValue_ + upperThreshold_)
	{
		return true;
	}
	else
	{
		return false;
	}
}


} /* namespace grid_map */
