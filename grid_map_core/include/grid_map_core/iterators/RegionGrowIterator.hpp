/*
 * RegionGrowIterator.hpp
 *
 *  Created on: Aug 22 2017
 *      Author: Tobias Meier, tmeier@anybotics.com
 *   Institute: ANYbotics AG
 */

#pragma once

#include "grid_map_core/GridMap.hpp"


namespace grid_map {

/*!
 * Iterator class to iterate through similar surrounding elements of the map.
 * Similarity is defined by a threshold.
 */
class RegionGrowIterator
{
public:

  /*!
   * Constructor.
   * @param gridMap:        The grid map to operate on.
   * @param start:          Starting position of the iteration.
   * @param lowerThreshold: Lower threshold value for the comparison.
   * @param upperThreshold: Upper threshold value for the comparison.
   */
  RegionGrowIterator(const GridMap& gridMap, const std::string& layer, const Position& start, const float lowerThreshold, const float upperThreshold);

  /*!
   * Assignment date from other operator.
   * @param iterator to copy data from.
   * @return a reference to *this.
   */
  RegionGrowIterator& operator =(const RegionGrowIterator& other);

  /*!
   * Compare to another operator.
   * @return wether the current iterator points to a different address than the other one.
   */
  bool operator !=(const RegionGrowIterator& other) const;

  /*!
   * Dereference operator.
   * @return the index to which the iterator is pointing.
   */
  const Index& operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  RegionGrowIterator& operator ++();

  bool isPastEnd();

private:

  void checkLine();

  /*!
   * Check ifvalue at current index is within the threshold.
   * @return true if within, false otherwise.
   */
  bool isWithinThreshold(const Index index) const;

  //! Data.
   const Matrix& data_;
   Length mapLength_;

  //! Current index.
  Index currentIndex_;

  //! Start index for current line.
  Index currentStartIndex_;


  //! Threshold values for cell comparison.
  double lowerThreshold_;
  double upperThreshold_;



  //!
  float& referenceValue_;
  std::vector<Index> futureStartIndices_;
  std::vector<Index> pastStartIndices_;
  Index tempIndex_;
  bool reverse_;
  bool complete_;


};

} /* namespace */
