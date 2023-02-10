//
// Created by stefan on 06.09.21.
//

#ifndef HECTOR_MATH_BOUNDED_VECTOR_H
#define HECTOR_MATH_BOUNDED_VECTOR_H

#include <array>
#include <assert.h>

namespace hector_math
{

template<typename T, int MaxSize>
class BoundedVector
{
public:
  using iterator = typename std::array<T, MaxSize>::iterator;
  using const_iterator = typename std::array<T, MaxSize>::const_iterator;

  size_t size() const { return size_; }

  // push and pop
  void push_back( T val )
  {
    if ( size_ == MaxSize )
      throw std::length_error( "Maximum size reached!" );
    items_[size_++] = val;
  }

  template<typename... Args>
  void emplace_back( Args... args )
  {
    if ( size_ == MaxSize )
      throw std::length_error( "Maximum size reached!" );
    items_[size_++] = T( args... );
  }

  void pop_back()
  {
    assert( size_ > 0 );
    back().~T();
    --size_;
  }

  void erase( const_iterator position )
  {
    assert( position - begin() < (long)( size_ ) );
    position->~T();
    iterator start = begin() + ( position - begin() );
    for ( auto it = position + 1; it != end(); ++it, ++start ) *start = std::move( *it );
    --size_;
  }

  void erase( const_iterator first, const_iterator last )
  {
    assert( first >= begin() );
    assert( last - begin() <= (long)( size_ ) );
    if ( first >= last )
      return;
    iterator start = begin() + ( first - begin() );
    for ( auto it = start; it != last; ++it ) it->~T();
    for ( auto it = last; it != end(); ++it, ++start ) *start = std::move( *it );
    size_ -= ( last - first );
  }

  void clear()
  {
    if ( std::is_trivially_destructible<T>::value )
      size_ = 0;
    else
      while ( size_ > 0 ) pop_back();
  }

  // front
  T &front() { return items_.front(); }

  const T &front() const { return items_.front(); }

  // back
  T &back() { return items_[size_ - 1]; }

  const T &back() const { return items_[size_ - 1]; }

  // begin
  iterator begin() { return items_.begin(); }

  const_iterator begin() const { return items_.begin(); }

  // end
  iterator end() { return items_.begin() + size_; }

  const_iterator end() const { return items_.begin() + size_; }

  // operator[]
  T &operator[]( size_t index ) { return items_[index]; }

  const T &operator[]( size_t index ) const { return items_[index]; }

  // data
  T *data() { return items_.data(); }

  const T *data() const { return items_.data(); }

  void reserve( size_t size )
  {
    assert( size <= MaxSize && "Bounded vector can not reserve more than max size!" );
    (void)size;
  }

private:
  std::array<T, MaxSize> items_;
  size_t size_ = 0;
};
} // namespace hector_math

#endif // HECTOR_MATH_BOUNDED_VECTOR_H
