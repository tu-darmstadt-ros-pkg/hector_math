//
// Created by aljoscha on 02.02.2023.
//

#ifndef HECTOR_MATH_RING_BUFFER_H
#define HECTOR_MATH_RING_BUFFER_H

#include <array>
#include <assert.h>
#include<type_traits>

namespace hector_math
{

template<typename T, int MaxSize>
class RingBuffer
{
public:
  // template parameter controls whether the iterator returns const references
  template<bool C>
  struct ring_iterator {
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = T;
    using pointer = T *;
    using reference = T &;
    using const_reference = T const &;

    ring_iterator( std::array<T, MaxSize> *items, size_t index ) : items_( items ), index_( index )
    {
    }

    template<bool Z = C, typename std::enable_if<( !Z ), int>::type * = nullptr>
    reference operator*()
    {
      return ( *items_ )[index_];
    }
    template<bool Z = C, typename std::enable_if<( Z ), int>::type * = nullptr>
    const_reference operator*() const
    {
      return ( *items_ )[index_];
    }
    template<bool Z = C, typename std::enable_if<( !Z ), int>::type * = nullptr>
    reference operator->() noexcept
    {
      return &( ( *items_ )[index_] );
    }
    template<bool Z = C, typename std::enable_if<( Z ), int>::type * = nullptr>
    const_reference operator->() noexcept
    {
      return &( ( *items_ )[index_] );
    }

    ring_iterator<C> &operator++()
    {
      index_ = ++index_ % MaxSize;
      return *this;
    }

    const ring_iterator<C> operator++( int )
    {
      auto tmp = *this;
      index_ = ++index_ % MaxSize;
      return tmp;
    }
    size_t index() { return index_; }
    friend bool operator==( const ring_iterator &a, const ring_iterator &b )
    {
      return a.index_ == b.index_;
    };
    friend bool operator!=( const ring_iterator &a, const ring_iterator &b )
    {
      return a.index_ != b.index_;
    };

  private:
    std::array<T, MaxSize> *items_; // array ?
    size_t index_;
  };

  using iterator = ring_iterator<false>;
  using const_iterator = ring_iterator<true>;

  size_t size() const { return size_; }

  // push and pop
  void push_back( T val )
  {
    items_[head_index] = val;
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
    if ( size_ < 0 )
      throw std::length_error( "RingBuffer is empty!" );
    T tmp = std::move( items_[tail_index] );
    pop_front();
    return tmp;
  }

  template<typename... Args>
  void emplace_back( Args... args )
  {
    items_[head_index] = T( args... );
    added_element_head_adapt_indices();
  }

  // begin "points" to the oldest element in ringbuffer
  iterator begin() noexcept { return iterator( &items_, tail_index ); }
  // end points to the first empty cell
  iterator end() noexcept { return iterator( &items_, head_index ); }

  // begin "points" to the oldest element in ringbuffer
  const_iterator cbegin() noexcept { return const_iterator( &items_, tail_index ); }
  // end points to the first empty cell
  const_iterator cend() noexcept { return const_iterator( &items_, head_index ); }

  // front
  T &front() { return items_[tail_index]; }

  const T &front() const { return items_[tail_index]; }

  // back
  T &back() { return items_[head_index - 1]; }

  const T &back() const { return items_[head_index - 1]; }

  void clear()
  {
    if ( std::is_trivially_destructible<T>::value ) {
      head_index = 0;
      tail_index = 0;
      size_ = 0;
    } else {
      while ( size_ > 0 ) pop_front();
    }
  }

  // const T &operator[]( size_t index ) const { return items_[index]; }

  bool full() { return size_ == MaxSize - 1; }

  bool empty() { return size_ == 0; }

private:
  void added_element_head_adapt_indices()
  {
    if ( full() ) {
      tail_index = ++tail_index % MaxSize;
      ; // overwrite oldest element
    } else {
      size_++;
    }
    head_index = ++head_index % MaxSize;
  }

  void removed_element_tail_adapt_indices()
  {
    tail_index = ++tail_index % MaxSize;
    size_--;
  }

  std::array<T, MaxSize> items_;
  size_t size_ = 0;
  size_t head_index = 0;
  size_t tail_index = 0;
};
} // namespace hector_math

#endif // HECTOR_MATH_RING_BUFFER_H
