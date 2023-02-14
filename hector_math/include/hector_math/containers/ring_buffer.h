//
// Created by aljoscha on 02.02.2023.
//

#ifndef HECTOR_MATH_RING_BUFFER_H
#define HECTOR_MATH_RING_BUFFER_H

#include <array>
#include <assert.h>
#include <type_traits>

namespace hector_math
{

template<typename T, int MaxSize>
class RingBuffer
{
public:
  // template parameter controls whether the iterator returns const references
  template<bool IS_CONST>
  struct ring_iterator {
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = T;
    using pointer = T *;
    using reference = T &;
    using const_reference = T const &;

    ring_iterator( RingBuffer<T, MaxSize> *buffer, size_t index )
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
      index_ = ++index_ % MaxSize;
      counter_++;
      return *this;
    }

    const ring_iterator<IS_CONST> operator++( int )
    {
      auto tmp = *this;
      index_ = ++index_ % MaxSize;
      counter_++;
      return tmp;
    }
    size_t index() { return index_; }

    //    Would work if size of array MaxSize + 1
    //    bool operator==(ring_iterator other){
    //      return &( ( *buffer_ )[index_] ) == &( ( *other.buffer_ )[other.index_] );
    //    }
    //
    //    bool operator!=(ring_iterator other){
    //      return &( ( *buffer_ )[index_] ) != &( ( *other.buffer_ )[other.index_] );
    //    }

    // Iterator must iterate from start to end, for every ++ counter increases until reaches size
    // the index the iterator points to is the same for start and end if the buffer is full
    template<bool IS_CONST_OTHER>
    bool operator==( ring_iterator<IS_CONST_OTHER> other )
    {
      return buffer_->size() == other.counter_ || counter_ == other.buffer_->size();
    }
    template<bool IS_CONST_OTHER>
    bool operator!=( ring_iterator<IS_CONST_OTHER> other )
    {
      return buffer_->size() != other.counter_ && counter_ != other.buffer_->size();
    }

  private:
    RingBuffer<T, MaxSize> *buffer_;
    size_t index_;
    size_t counter_ = 0;
  };

  using iterator = ring_iterator<false>;
  using const_iterator = ring_iterator<true>;

  size_t size() const { return size_; }

  // push and pop
  void push_back( T val )
  {
    items_[head_index_] = val;
    added_element_head_adapt_indices();
  }

  void pop_front()
  {
    if ( size_ > 0 ) {
      front().~T(); // front is last element -> oldest element
      removed_element_tail_adapt_indices();
    }
  }

  T read_and_pop_front()
  {
    if ( size_ <= 0 )
      throw std::length_error( "RingBuffer is empty!" );
    T tmp = std::move( items_[get_tail_index()] );
    removed_element_tail_adapt_indices();
    return tmp;
  }

  template<typename... Args>
  void emplace_back( Args... args )
  {
    items_[head_index_] = T( args... );
    added_element_head_adapt_indices();
  }

  // begin "points" to the oldest element in ringbuffer
  iterator begin() noexcept { return iterator( this, get_tail_index() ); }
  // end points to the first empty cell
  iterator end() noexcept { return iterator( this, head_index_ ); }

  // begin "points" to the oldest element in ringbuffer
  const_iterator cbegin() noexcept { return const_iterator( this, get_tail_index() ); }
  // end points to the first empty cell
  const_iterator cend() noexcept { return const_iterator( this, head_index_ ); }

  // front
  T &front() { return items_[get_tail_index()]; }

  const T &front() const { return items_[get_tail_index()]; }

  // back
  T &back() { return items_[head_index_ - 1]; }

  const T &back() const { return items_[head_index_ - 1]; }

  void clear()
  {
    if ( std::is_trivially_destructible<T>::value ) {
      head_index_ = 0;
      size_ = 0;
    } else {
      while ( size_ > 0 ) pop_front();
    }
  }

  bool full() { return size_ == MaxSize; }

  bool empty() { return size_ == 0; }

private:
  const T &operator[]( size_t index ) const { return items_[index]; }

  T &operator[]( size_t index ) { return items_[index]; }

  void added_element_head_adapt_indices()
  {
    if ( !full() ) {
      size_++;
    }
    head_index_ = ++head_index_ % MaxSize;
  }

  void removed_element_tail_adapt_indices() { size_--; }
  size_t get_tail_index() { return ( head_index_ - size_ + MaxSize ) % MaxSize; }

  std::array<T, MaxSize> items_;
  size_t size_ = 0;
  size_t head_index_ = 0;
};
} // namespace hector_math

#endif // HECTOR_MATH_RING_BUFFER_H
