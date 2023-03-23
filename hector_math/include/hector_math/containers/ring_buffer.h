// Copyright (c) 2021 Aljoscha Schmidt, Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_RING_BUFFER_H
#define HECTOR_MATH_RING_BUFFER_H

#include <array>
#include <cassert>
#include <type_traits>

namespace hector_math
{

/*!
 * A RingBuffer is a limited size storage container that once full will overwrite the oldest
 * elements when a new element is added.
 * @tparam T The type of the elements stored in this RingBuffer.
 * @tparam Nm The maximum number of elements stored in the RingBuffer at any time.
 */
template<typename T, size_t N>
class RingBuffer
{
public:
  static constexpr size_t Size = N;
  using value_type = T;
  using pointer = value_type *;
  using const_pointer = const value_type *;
  using reference = value_type &;
  using const_reference = const value_type &;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;

  // template parameter controls whether the iterator returns const references
  template<bool IS_CONST>
  struct ring_iterator;
  using iterator = ring_iterator<false>;
  using const_iterator = ring_iterator<true>;

  //! @returns true if the container is empty, false otherwise
  constexpr bool empty() const { return size_ == 0; }
  //! @returns true if the container is full, false otherwise. Appending to a full container will overwrite old elements.
  bool full() const { return size_ == Size; }
  //! @returns the number of elements
  constexpr size_t size() const { return size_; }
  //! @returns the maximum number of elements which is equal to the template parameter TSize.
  constexpr size_t capacity() const { return Size; }
  //! @returns the maximum number of elements which is equal to the template parameter TSize.
  constexpr size_t max_size() const { return Size; }

  /*!
   * Adds an element to the end of the RingBuffer.
   * If the RingBuffer is full the oldest element will be overwritten.
   * @param value element to be appended.
   */
  void push_back( const_reference value )
  {
    items_[tail_index_] = value;
    added_element_tail_adapt_indices();
  }

  //! Deletes the oldest element in the RingBuffer.
  void pop_front()
  {
    if ( size_ > 0 ) {
      front().~value_type(); // front is last element -> oldest element
      removed_element_at_head_adapt_indices();
    }
  }

  /*!
   * Reads and deletes the oldest element of the RingBuffer.
   * @return the oldest not yet overwritten element from The RingBuffer.
   */
  value_type read_and_pop_front()
  {
    if ( size_ <= 0 )
      throw std::length_error( "RingBuffer is empty!" );
    value_type tmp = std::move( items_[get_head_index()] );
    removed_element_at_head_adapt_indices();
    return tmp;
  }
  /*!
   * Constructs an element and appends it to the RingBuffer.
   * If the RingBuffer is already full, the oldest element is overwritten.
   * @param args The arguments that are passed to the constructor of the element.
   */
  template<typename... Args>
  void emplace_back( Args &&...args )
  {
    items_[tail_index_] = value_type( std::forward<Args>( args )... );
    added_element_tail_adapt_indices();
  }

  //! @returns an iterator pointing to the oldest element in the ringbuffer.
  iterator begin() noexcept { return iterator( this, get_head_index(), 0 ); }

  //! @returns an iterator pointing to the position after the newest element in the ringbuffer.
  iterator end() noexcept { return iterator( this, get_head_index(), size_ ); }

  //! @returns a const iterator pointing to the oldest element in the ringbuffer.
  const_iterator cbegin() noexcept { return const_iterator( this, get_head_index() ); }

  //! @returns a const iterator pointing to the position after the newest element in the ringbuffer.
  const_iterator cend() noexcept { return const_iterator( this, get_head_index(), size_ ); }

  //! @returns a reference to the oldest element in the buffer.
  reference front() { return items_[get_head_index()]; }
  //! @returns a const reference to the oldest element in the buffer.
  const_reference front() const { return items_[get_head_index()]; }

  //! @returns a reference to the newest element in the buffer.
  reference back() { return items_[get_last_index()]; }
  //! @returns a const reference to the newest element in the buffer.
  const_reference back() const { return items_[get_last_index()]; }

  //! Clears the contents of the RingBuffer.
  void clear()
  {
    if ( std::is_trivially_destructible<value_type>::value ) {
      tail_index_ = 0;
      size_ = 0;
    } else {
      while ( size_ > 0 ) pop_front();
    }
  }

  const_reference operator[]( size_t index ) const { return items_[get_normalised_index( index )]; }

  reference operator[]( size_t index ) { return items_[get_normalised_index( index )]; }

private:
  // normalises index to be as if head would reside at position 0
  size_t get_normalised_index( size_t index ) const
  {
    const size_t result = get_head_index() + index;
    return result < Size ? result : result - Size;
  }

  void added_element_tail_adapt_indices()
  {
    if ( !full() )
      size_++;
    tail_index_ = tail_index_ == ( Size - 1 ) ? 0 : ( tail_index_ + 1 );
  }

  void removed_element_at_head_adapt_indices() { size_--; }

  //! The index of the oldest element
  size_t get_head_index() const
  {
    return tail_index_ >= size_ ? ( tail_index_ - size_ ) : ( tail_index_ + Size - size_ );
  }
  size_t get_last_index() const { return tail_index_ == 0 ? ( Size - 1 ) : ( tail_index_ - 1 ); }
  std::array<value_type, Size> items_;
  size_t size_ = 0;
  size_t tail_index_ = 0;
};

template<typename T, size_t TSize>
template<bool IS_CONST>
struct RingBuffer<T, TSize>::ring_iterator {
  using iterator_category = std::random_access_iterator_tag;
  using difference_type = std::ptrdiff_t;
  // using difference_type = typename std::iterator<std::random_access_iterator_tag, T>::difference_type;
  using value_type = T;
  using pointer = typename std::conditional<IS_CONST, const T *, T *>::type;
  using reference = typename std::conditional<IS_CONST, const T &, T &>::type;
  using const_reference = T const &;

  ring_iterator( RingBuffer<T, TSize> *buffer, int offset, int index = 0 )
      : buffer_( buffer ), offset_( offset ), buffer_index_( ( offset + index ) % TSize ),
        iterator_index_( index )
  {
  }

  reference operator*() noexcept { return buffer_->items_[buffer_index_]; }
  const_reference operator*() const noexcept { return buffer_->items_[buffer_index_]; }
  reference operator->() noexcept { return &( buffer_->items_[buffer_index_] ); }
  const_reference operator->() const noexcept { return &( buffer_->items_[buffer_index_] ); }

  ring_iterator<IS_CONST> &operator++()
  {
    buffer_index_ = buffer_index_ == ( Size - 1 ) ? 0 : ( buffer_index_ + 1 );
    ++iterator_index_;
    return *this;
  }

  ring_iterator<IS_CONST> &operator--()
  {
    buffer_index_ = buffer_index_ == 0 ? ( Size - 1 ) : ( buffer_index_ - 1 );
    --iterator_index_;
    return *this;
  }

  // NOLINTNEXTLINE(cert-dcl21-cpp) lvalue ref-qualify to prevent (it++)++ mistakes without const.
  ring_iterator<IS_CONST> operator++( int ) &
  {
    auto tmp = *this;
    ++( *this );
    return tmp;
  }

  ring_iterator<IS_CONST> operator--( int ) & // NOLINT(cert-dcl21-cpp)
  {
    auto tmp = *this;
    --( *this );
    return tmp;
  }

  difference_type operator-( const ring_iterator<IS_CONST> &other ) const
  {
    assert( buffer_ == other.buffer_ && "Iterators have to point to the same buffer!" );
    assert( offset_ == other.offset_ && "You are performing arithmetic with two iterators from "
                                        "different head points. This indicates a bug." );
    return iterator_index_ - other.iterator_index_;
  }

  ring_iterator<IS_CONST> operator+( difference_type nums ) const
  {
    return ring_iterator<IS_CONST>( buffer_, offset_, iterator_index_ + nums );
  }

  ring_iterator<IS_CONST> operator-( difference_type nums ) const
  {
    return ring_iterator<IS_CONST>( buffer_, offset_, iterator_index_ - nums );
  }

  operator ring_iterator<true>() const // NOLINT(google-explicit-constructor)
  {
    return ring_iterator<true>( buffer_, offset_, iterator_index_ );
  }

  template<bool IS_CONST_OTHER>
  bool operator==( const ring_iterator<IS_CONST_OTHER> &other ) const
  {
    assert( buffer_ == other.buffer_ &&
            "You are comparing two iterators from different ring buffers. This indicates a bug." );
    assert( offset_ == other.offset_ &&
            "You are comparing two iterators from different head points. This indicates a bug." );
    return iterator_index_ == other.iterator_index_;
  }

  template<bool IS_CONST_OTHER>
  bool operator!=( const ring_iterator<IS_CONST_OTHER> &other ) const
  {
    assert( buffer_ == other.buffer_ &&
            "You are comparing two iterators from different ring buffers. This indicates a bug." );
    assert( offset_ == other.offset_ &&
            "You are comparing two iterators from different head points. This indicates a bug." );
    return iterator_index_ != other.iterator_index_;
  }

private:
  RingBuffer<T, TSize> *buffer_;
  int offset_;
  int iterator_index_;
  int buffer_index_;
};
} // namespace hector_math

#endif // HECTOR_MATH_RING_BUFFER_H
