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
    using iterator_category = std::random_access_iterator_tag;
    using difference_type = std::ptrdiff_t;
    // using difference_type = typename std::iterator<std::random_access_iterator_tag, T>::difference_type;
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
      index_++;
      if(index_>MaxSize)index_=0;
      if (!is_index_valid()){
        index_=buffer_->get_head_index();
      }
      return *this;
    }

    ring_iterator<IS_CONST> &operator--()
    {
      index_= (index_ + MaxSize)% (MaxSize+1);// index-- and back to 0 if > MaxSize
      if (!is_index_valid()){
        index_=buffer_->tail_index_;
      }
      return *this;
    }

    const ring_iterator<IS_CONST> operator++( int )
    {
      auto tmp = *this;
      index_++;
      if(index_>MaxSize)index_=0;
      if (!is_index_valid()){
        index_=buffer_->get_head_index();
      }
      return tmp;
    }

    const ring_iterator<IS_CONST> operator--( int )
    {
      auto tmp = *this;
      index_= (index_ + MaxSize)% (MaxSize+1);// index-- and back to 0 if > MaxSize
      if (!is_index_valid()){
        index_=buffer_->tail_index_;
      }
      return tmp;
    }

    difference_type operator-( const ring_iterator<IS_CONST> &other ) const
    {
      difference_type diff = 0;
      if(one_valid_region()){
        diff = get_difference(index_,other.index_);
      }else{
        if((other.index_ < buffer_->get_head_index()&&index_ < buffer_->get_head_index()) ||
             (other.index_>buffer_->tail_index_ && index_ >buffer_->tail_index_)){
          diff = get_difference(index_, other.index_);
        }else{
          if(other.index_>index_){
            diff = index_ + MaxSize+1 - other.index_;
          }else{
            diff = other.index_ + MaxSize+1 - index_;
          }
        }
      }
      return diff;
    }
    ring_iterator<IS_CONST> operator+( difference_type nums ) const
    {
      if (nums>=buffer_->size_)nums = nums%buffer_->size_;
      size_t index;
     if(one_valid_region()){
        size_t distance_to_tail = buffer_->tail_index_ - index_;
        if (distance_to_tail>=nums){
          index = index_ + nums;
        }else{
          index = buffer_->get_head_index() + (nums - distance_to_tail -1);
        }
      }else{
        index = index_;
        while(nums>0){
          if(index > buffer_->tail_index_){
            size_t distance_to_array_end= MaxSize - index;
            if(distance_to_array_end>=nums){
              index += nums;
              nums = 0;
            }else{
              index = 0;
              nums -= (distance_to_array_end+1);
            }
          }else{
            size_t distance_to_tail = buffer_->tail_index_ - index;
            if (distance_to_tail>=nums){
              index = index + nums;
              nums = 0;
            }else{
              index = buffer_->get_head_index();
              nums = nums - distance_to_tail -1;
            }
          }
        }
      }
      return ring_iterator<IS_CONST>(buffer_,index );
    }
    ring_iterator<IS_CONST> operator-( int nums ) const
    {
      ring_iterator<IS_CONST> t = ring_iterator<IS_CONST>(buffer_, index_);
      for(int i=0;i<nums;i++) --t;
      return t;
    }
    //friend ring_iterator<IS_CONST> operator+( difference_type nums,
    //                                          const ring_iterator<IS_CONST> &other )
    //{
    //  return ring_iterator<IS_CONST>(other.buffer_, ( nums + other.index_ ) % ( MaxSize + 1 ) );
    //}
    //friend ring_iterator<IS_CONST> operator-( difference_type nums,
    //                                          const ring_iterator<IS_CONST> &other )
    //{
    //  return ring_iterator<IS_CONST>(other.buffer_, ( nums - other.index_ + MaxSize + 1 ) % ( MaxSize + 1 ) );
    //}

    size_t index() const { return index_; }

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

  private:
    bool is_index_valid()const{
      // must be between head and tail
      if(one_valid_region()){
        return index_>= buffer_->get_head_index() && index_<= buffer_->tail_index_;
      }
      return !(index_> buffer_->tail_index_ && index_< buffer_->get_head_index()) && index_>=0 and index_<=MaxSize;
    }

    bool one_valid_region() const{
        return buffer_->get_head_index() < buffer_->tail_index_;
    }
    difference_type get_difference(size_t a, size_t b)const{
      difference_type  diff;
      if(a>b){
        diff = a - b;
      }else{
        diff = b - a;
      }
      return diff;
    }
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
   * Returns the maximum number of elements that the RingBuffer can store.
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
      removed_element_head_adapt_indices();
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
    removed_element_head_adapt_indices();
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
   * end points to the first empty cell
   * @return iterator to the newest element
   */
  iterator end() noexcept { return iterator( this, get_next_tail_index() ); }

  /*!
   * begin "points" to the oldest element in ringbuffer
   * @return a const_iterator the oldest element
   */
  const_iterator cbegin() noexcept { return const_iterator( this, get_head_index() ); }

  /*!
   *  end points to the first empty cell
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
      tail_index_ = MaxSize;
      size_ = 0;
    } else {
      while ( size_ > 0 ) pop_front();
    }
  }
  /*!
   * @return if the RingBuffer is already full.
   */
  bool full() const { return size_ == MaxSize; }
  /*!
   * @return if the buffer contains no elements
   */
  bool empty() const { return size_ == 0; }

  const T &operator[]( size_t index ) const { return items_[index]; }

  T &operator[]( size_t index ) { return items_[index]; }

private:
  void added_element_tail_adapt_indices()
  {
    if ( !full() ) {
      size_++;
    }
    tail_index_ = ++tail_index_ % items_.size();
  }

  void removed_element_head_adapt_indices() { size_--; }
  size_t get_head_index() { return ( tail_index_ - size_ + items_.size() + 1 ) % items_.size(); }
  size_t get_next_tail_index() { return ( tail_index_ + 1 ) % items_.size(); }
  std::array<T, MaxSize + 1> items_;
  size_t size_ = 0;
  size_t tail_index_ = MaxSize;
};
} // namespace hector_math

#endif // HECTOR_MATH_RING_BUFFER_H
