// Copyright (c) 2024 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_AGGREGATORS_H
#define HECTOR_MATH_AGGREGATORS_H

namespace hector_math
{

template<typename Scalar>
class MeanAggregator
{
public:
  MeanAggregator() = default;

  //! Adds a guaranteed finite / valid value to the aggregator.
  void add( const Scalar &value )
  {
    sum_ += value;
    ++count_;
  }

  //! Returns the mean of all added values. If no values have been added, 0 is returned.
  Scalar mean() const { return count_ > 0 ? sum_ / count_ : 0; }

  //! Clears the aggregator.
  void clear()
  {
    sum_ = 0;
    count_ = 0;
  }

  //! Returns true if no values have been added.
  bool empty() const { return count_ == 0; }

  //! Returns the number of values that have been added.
  long count() const { return count_; }

private:
  Scalar sum_ = 0;
  long count_ = 0;
};

//! Similar to MeanAggregator except that it is robust to non-finite values.
template<typename Scalar>
class RobustMeanAggregator
{
public:
  RobustMeanAggregator() = default;

  //! Adds a value to the aggregator. If the value is not finite, it is ignored.
  void add( const Scalar &value )
  {
    if ( !std::isfinite( value ) )
      return;
    sum_ += value;
    ++count_;
  }

  //! Adds a guaranteed finite value to the aggregator.
  void addFinite( const Scalar &value )
  {
    sum_ += value;
    ++count_;
  }

  //! Returns the mean of all added values. If no values have been added, 0 is returned.
  Scalar mean() const { return count_ > 0 ? sum_ / count_ : 0; }

  //! Clears the aggregator.
  void clear()
  {
    sum_ = 0;
    count_ = 0;
  }

  //! Returns true if no values have been added.
  bool empty() const { return count_ == 0; }

  //! Returns the number of values that have been added.
  long count() const { return count_; }

private:
  Scalar sum_ = 0;
  long count_ = 0;
};
} // namespace hector_math

#endif // HECTOR_MATH_AGGREGATORS_H
