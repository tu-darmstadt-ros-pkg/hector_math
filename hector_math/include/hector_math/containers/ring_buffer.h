// Copyright (c) 2021 Aljoscha Schmidt, Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_RING_BUFFER_H
#define HECTOR_MATH_RING_BUFFER_H

#include <array>
#include <assert.h>
#include <type_traits>

namespace hector_math
{

template<typename T, int ElementSize>
class RingBuffer
{
public:
  // template parameter controls whether the iterator returns const references
  template<bool IS_CONST>
  struct ring_iterator {
    using iterator_category = std::random_access_iterator_tag;
    using difference_type = std::ptrdiff_t;
    // using difference_type = typename std::iterator<std::random_access_iterator_tag, T>::difference_type;
    using value_type = T;
    using pointer = std::conditional_t<IS_CONST, const T *, T *>;
    using reference = std::conditional_t<IS_CONST, const T &, T &>;
    using const_reference = T const &;

    ring_iterator( RingBuffer<T, ElementSize> *buffer, size_t index )
        : buffer_( buffer ), index_( index )

    {
    }

    template<bool Z = IS_CONST, typename std::enable_if<( !Z ), int>::type * = nullptr>
    reference operator*() noexcept
    {
      return ( *buffer_ )[index_];
    }
    const_reference operator*() const noexcept { return ( *buffer_ )[index_]; }
    template<bool Z = IS_CONST, typename std::enable_if<( !Z ), int>::type * = nullptr>
    reference operator->() noexcept
    {
      return &( ( *buffer_ )[index_] );
    }
    const_reference operator->() const noexcept { return &( ( *buffer_ )[index_] ); }

    ring_iterator<IS_CONST> &operator++()
    {
      index_++;
      if ( index_ > ElementSize )
        index_ = 0;
      return *this;
    }

    ring_iterator<IS_CONST> &operator--()
    {
      index_ = ( index_ + ElementSize ) % StorageSize; // index-- and back to 0 if > ElementSize
      return *this;
    }

    const ring_iterator<IS_CONST> operator++( int )
    {
      auto tmp = *this;
      index_++;
      if ( index_ > ElementSize )
        index_ = 0;
      return tmp;
    }

    const ring_iterator<IS_CONST> operator--( int )
    {
      auto tmp = *this;
      index_ = ( index_ + ElementSize ) % StorageSize; // index-- and back to 0 if > ElementSize

      return tmp;
    }

    difference_type operator-( const ring_iterator<IS_CONST> &other ) const
    {
      // current index - other index
      difference_type diff = get_normalised_index() - other.get_normalised_index();
      return diff;
    }
    ring_iterator<IS_CONST> operator+( difference_type nums ) const
    {
      size_t index = ( index_ + nums ) % StorageSize;
      return ring_iterator<IS_CONST>( buffer_, index );
    }
    ring_iterator<IS_CONST> operator-( difference_type nums ) const
    {
      nums = nums % StorageSize;
      nums = StorageSize - nums;
      size_t index = ( index_ + nums ) % StorageSize;
      return ring_iterator<IS_CONST>( buffer_, index );
    }

    size_t index() const { return index_; }

    template<bool IS_CONST_OTHER>
    bool operator==( const ring_iterator<IS_CONST_OTHER> &other ) const
    {
      return buffer_ == other.buffer_ && index_ == other.index_;
    }
    template<bool IS_CONST_OTHER>
    bool operator!=( const ring_iterator<IS_CONST_OTHER> &other ) const
    {
      return !( buffer_ == other.buffer_ && index_ == other.index_ );
    }

  private:
    // normalises index to be as if head would reside at position 0
    size_t get_normalised_index() const
    {
      size_t index = ( index_ + StorageSize ) - buffer_->get_head_index();
      if ( index >= StorageSize )
        index -= StorageSize;
      return index;
    }
    RingBuffer<T, ElementSize> *buffer_;
    size_t index_;
  };

  using iterator = ring_iterator<false>;
  using const_iterator = ring_iterator<true>;

  /*!
   * The current amount of elements in the buffer. Zero if empty, ElementSize if full.
   * @return the current number of elements
   */
  size_t size() const { return size_; }
  /*!
   * Returns the maximum number of elements that the RingBuffer can store.
   * Controlled by the template argument ElementSize.
   * @return
   */
  size_t capacity() const { return ElementSize; };

  /*!
   * Appends an element to the RingBuffer. Elements are appended to the RingBuffer.
   * If the RingBuffer is full the oldest element will be overwritten!
   * @param val element to be appended
   */
  void push_back( T val )
  {
    added_element_tail_adapt_indices();
    items_[tail_index_] = val;
  }
  /*!
   * Deletes the oldest element in the RingBuffer.
   */
  void pop_front()
  {
    if ( size_ > 0 ) {
      front().~T(); // front is last element -> oldest element
      removed_element_at_head_adapt_indices();
    }
  }
  /*!
   * Reads and deletes the oldest element of the RingBuffer.
   * @return the oldest not yet overwritten element from The RingBuffer.
   */
  T read_and_pop_front()
  {
    if ( size_ <= 0 )
      throw std::length_error( "RingBuffer is empty!" );
    T tmp = std::move( items_[get_head_index()] );
    removed_element_at_head_adapt_indices();
    return tmp;
  }
  /*!
   * Constructs an element and appends a new element to the RingBuffer.
   * If the RingBuffer is already full, it overwrites the oldest element!
   * @tparam Args The element type to be stored.
   * @param args
   */
  template<typename... Args>
  void emplace_back( Args... args )
  {
    added_element_tail_adapt_indices();
    items_[tail_index_] = T( args... );
  }

  /*!
   * begin "points" to the oldest element in ringbuffer
   */
  iterator begin() noexcept { return iterator( this, get_head_index() ); }
  /*!
   * end points to the first empty cell, next to the newest element in the buffer
   * @return iterator to the newest element
   */
  iterator end() noexcept { return iterator( this, get_next_tail_index() ); }

  /*!
   * begin "points" to the oldest element in ringbuffer
   * @return a const_iterator the oldest element
   */
  const_iterator cbegin() noexcept { return const_iterator( this, get_head_index() ); }

  /*!
   *  end points to the first empty cell, next to the newest element in the buffer
   * @return  a const_iterator the newest element
   */
  const_iterator cend() noexcept { return const_iterator( this, get_next_tail_index() ); }

  /*!
   * @return A reference to the oldest element in the buffer
   */
  T &front() { return items_[get_head_index()]; }
  /*!
   * @return A const reference to the oldest element in the buffer
   */
  const T &front() const { return items_[get_head_index()]; }

  /*!
   * @return A reference to the newest element in the buffer
   */
  T &back() { return items_[tail_index_]; }
  /*!
   * @return A const reference to the newest element in the buffer
   */
  const T &back() const { return items_[tail_index_]; }
  /*!
   * Clears the Ringbuffer. If necessary deconstructs all ojbects in the buffer.
   */
  void clear()
  {
    if ( std::is_trivially_destructible<T>::value ) {
      tail_index_ = ElementSize;
      size_ = 0;
    } else {
      while ( size_ > 0 ) pop_front();
    }
  }
  /*!
   * @return if the RingBuffer is already full.
   */
  bool full() const { return size_ == ElementSize; }
  /*!
   * @return if the buffer contains no elements
   */
  bool empty() const { return size_ == 0; }

  const T &operator[]( size_t index ) const { return items_[index]; }

  T &operator[]( size_t index ) { return items_[index]; }
  static constexpr int StorageSize = ElementSize + 1;

private:
  void added_element_tail_adapt_indices()
  {
    if ( !full() ) {
      size_++;
    }
    tail_index_ = ++tail_index_ % items_.size();
  }

  void removed_element_at_head_adapt_indices() { size_--; }
  size_t get_head_index() const
  {
    return ( tail_index_ + items_.size() + 1 - size_ ) % items_.size();
  }
  size_t get_next_tail_index() const { return ( tail_index_ + 1 ) % items_.size(); }
  std::array<T, StorageSize> items_;
  size_t size_ = 0;
  size_t tail_index_ = ElementSize;
};
} // namespace hector_math

#endif // HECTOR_MATH_RING_BUFFER_H
