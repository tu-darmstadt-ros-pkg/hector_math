// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_BOUNDED_VECTOR_H
#define HECTOR_MATH_BOUNDED_VECTOR_H

#include <array>
#include <cassert>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace hector_math
{

//! Fixed-capacity vector storing its elements inline in a std::array.
//! All MaxSize elements are alive for the entire lifetime of the container, hence T has to be
//! default constructible. Removed elements are reset to a value-initialized T so they release any
//! resources they hold.
template<typename T, int MaxSize>
class BoundedVector
{
public:
  using iterator = typename std::array<T, MaxSize>::iterator;
  using const_iterator = typename std::array<T, MaxSize>::const_iterator;

  //! @returns the number of elements in the container.
  [[nodiscard]] std::size_t size() const { return size_; }
  //! @returns true if the container is empty, false otherwise.
  [[nodiscard]] bool empty() const { return size_ == 0; }
  //! @returns true if the container is full, false otherwise. Appending to a full container will throw.
  [[nodiscard]] bool full() const { return size_ == MaxSize; }

  // push and pop
  void push_back( const T &val )
  {
    if ( size_ == MaxSize )
      throw std::length_error( "Maximum size reached!" );
    items_[size_++] = val;
  }

  void push_back( T &&val )
  {
    if ( size_ == MaxSize )
      throw std::length_error( "Maximum size reached!" );
    items_[size_++] = std::move( val );
  }

  template<typename... Args>
  void emplace_back( Args &&...args )
  {
    if ( size_ == MaxSize )
      throw std::length_error( "Maximum size reached!" );
    if constexpr ( sizeof...( Args ) == 1 && ( std::is_same_v<std::decay_t<Args>, T> && ... ) )
      items_[size_++] = ( std::forward<Args>( args ), ... );
    else
      items_[size_++] = T( std::forward<Args>( args )... );
  }

  iterator insert( const_iterator position, const T &val )
  {
    assert( position >= begin() );
    assert( position - begin() <= (long)( size_ ) );
    if ( size_ == MaxSize )
      throw std::length_error( "Maximum size reached!" );
    T tmp = val;
    for ( iterator it = end(); it != position; --it ) *it = std::move( *( it - 1 ) );
    auto dst = begin() + ( position - begin() ); // remove the const
    *dst = std::move( tmp );
    ++size_;
    return dst;
  }

  void pop_back()
  {
    assert( size_ > 0 );
    reset( size_ - 1, size_ );
    --size_;
  }

  void erase( const_iterator position )
  {
    assert( position >= begin() );
    assert( position - begin() < (long)( size_ ) );
    iterator dst = begin() + ( position - begin() );
    for ( iterator src = dst + 1; src != end(); ++src, ++dst ) *dst = std::move( *src );
    reset( size_ - 1, size_ );
    --size_;
  }

  void erase( const_iterator first, const_iterator last )
  {
    assert( first >= begin() );
    assert( last - begin() <= (long)( size_ ) );
    if ( first >= last )
      return;
    iterator dst = begin() + ( first - begin() );
    for ( iterator src = begin() + ( last - begin() ); src != end(); ++src, ++dst )
      *dst = std::move( *src );
    const std::size_t count = last - first;
    reset( size_ - count, size_ );
    size_ -= count;
  }

  void clear()
  {
    reset( 0, size_ );
    size_ = 0;
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
  T &operator[]( std::size_t index ) { return items_[index]; }

  const T &operator[]( std::size_t index ) const { return items_[index]; }

  // data
  T *data() { return items_.data(); }

  const T *data() const { return items_.data(); }

  void reserve( std::size_t size )
  {
    assert( size <= MaxSize && "Bounded vector can not reserve more than max size!" );
    (void)size;
  }

  //! Elements added by growing the container are value-initialized.
  void resize( std::size_t size )
  {
    if ( size > MaxSize )
      throw std::length_error( std::to_string( size ) +
                               " is greater than maximum size: " + std::to_string( MaxSize ) );
    for ( std::size_t i = size_; i < size; ++i ) items_[i] = T();
    reset( size, size_ );
    size_ = size;
  }

private:
  //! Resets the elements in [first, last) to release the resources they hold.
  //! Does nothing for types that do not own resources.
  void reset( std::size_t first, std::size_t last )
  {
    if constexpr ( !std::is_trivially_destructible_v<T> ) {
      for ( std::size_t i = first; i < last; ++i ) items_[i] = T();
    }
  }

  std::array<T, MaxSize> items_;
  std::size_t size_ = 0;
};
} // namespace hector_math

#endif // HECTOR_MATH_BOUNDED_VECTOR_H
