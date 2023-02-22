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
      index_ = ++index_ % ( MaxSize + 1 );
      return *this;
    }

    const ring_iterator<IS_CONST> operator++( int )
    {
      auto tmp = *this;
      index_ = ++index_ % ( MaxSize + 1 );
      return tmp;
    }
    size_t index() { return index_; }

    // Would work if size of array MaxSize + 1
    template<bool IS_CONST_OTHER>
    bool operator==( const ring_iterator<IS_CONST_OTHER> &other ) const
    {
      return &( ( *buffer_ )[index_] ) == &( ( *other.buffer_ )[other.index_] );
    }
    template<bool IS_CONST_OTHER>
    bool operator!=( const ring_iterator<IS_CONST_OTHER> &other ) const
    {
      return &( ( *buffer_ )[index_] ) != &( ( *other.buffer_ )[other.index_] );
    }

    // Iterator must iterate from start to end, for every ++ counter increases until reaches size
    // the index the iterator points to is the same for start and end if the buffer is full
    //    template<bool IS_CONST_OTHER>
    //    bool operator==( ring_iterator<IS_CONST_OTHER> other )
    //    {
    //      return counter_ == other.counter_ ;
    //    }
    //    template<bool IS_CONST_OTHER>
    //    bool operator!=( ring_iterator<IS_CONST_OTHER> other )
    //    {
    //      return counter_!= other.counter_;
    //    }

  private:
    RingBuffer<T, MaxSize> *buffer_;
    size_t index_;
  };

  using iterator = ring_iterator<false>;
  using const_iterator = ring_iterator<true>;

  /*!
   * The current amount of elements in the buffer. Zero if empty, MaxSize if full.
   * @return the current number of elements
   */
  size_t size() const { return size_; }
  /*!
   * Returns the maximum number of elemts that the RingBuffer can store.
   * Controlled by the template argument MaxSize.
   * @return
   */
  size_t capacity() const { return MaxSize; };

  /*!
   * Appends an element to the RingBuffer. Elements are appended to the RingBuffer.
   * If the RingBuffer is full the oldest element will be overwritten!
   * @param val element to be appended
   */
  void push_back( T val )
  {
    items_[head_index_] = val;
    added_element_head_adapt_indices();
  }
  /*!
   * Deletes the oldest element in the RingBuffer.
   */
  void pop_front()
  {
    if ( size_ > 0 ) {
      front().~T(); // front is last element -> oldest element
      removed_element_tail_adapt_indices();
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
    T tmp = std::move( items_[get_tail_index()] );
    removed_element_tail_adapt_indices();
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
    items_[head_index_] = T( args... );
    added_element_head_adapt_indices();
  }

  /*!
   * begin "points" to the oldest element in ringbuffer
   */
  iterator begin() noexcept { return iterator( this, get_tail_index() ); }
  /*!
   * end points to the first empty cell
   * @return iterator to the newest element
   */
  iterator end() noexcept { return iterator( this, head_index_ ); }

  /*!
   * begin "points" to the oldest element in ringbuffer
   * @return a const_iterator the oldest element
   */
  const_iterator cbegin() noexcept { return const_iterator( this, get_tail_index() ); }

  /*!
   *  end points to the first empty cell
   * @return  a const_iterator the newest element
   */
  const_iterator cend() noexcept { return const_iterator( this, head_index_ ); }

  /*!
   * @return A reference to the newest element in the buffer
   */
  T &front() { return items_[get_tail_index()]; }
  /*!
   * @return A const reference to the newest element in the buffer
   */
  const T &front() const { return items_[get_tail_index()]; }

  /*!
   * @return A reference to the oldest element in the buffer
   */
  T &back() { return items_[get_1_before_head()]; }
  /*!
   * @return A const reference to the oldest element in the buffer
   */
  const T &back() const { return items_[get_1_before_head()]; }
  /*!
   * Clears the Ringbuffer. If necessary deconstructs all ojbects in the buffer.
   */
  void clear()
  {
    if ( std::is_trivially_destructible<T>::value ) {
      head_index_ = 0;
      size_ = 0;
    } else {
      while ( size_ > 0 ) pop_front();
    }
  }
  /*!
   * @return if the RingBuffer is already full.
   */
  bool full() { return size_ == MaxSize; }
  /*!
   * @return if the buffer contains no elements
   */
  bool empty() { return size_ == 0; }

  const T &operator[]( size_t index ) const { return items_[index]; }

  T &operator[]( size_t index ) { return items_[index]; }

private:
  void added_element_head_adapt_indices()
  {
    if ( !full() ) {
      size_++;
    }
    head_index_ = ++head_index_ % items_.size();
  }

  void removed_element_tail_adapt_indices() { size_--; }
  size_t get_tail_index() { return ( head_index_ - size_ + items_.size() ) % items_.size(); }
  size_t get_1_before_head() { return ( head_index_ - 1 + items_.size() ) % items_.size(); }

  std::array<T, MaxSize + 1> items_;
  size_t size_ = 0;
  size_t head_index_ = 0;
};
} // namespace hector_math

#endif // HECTOR_MATH_RING_BUFFER_H
