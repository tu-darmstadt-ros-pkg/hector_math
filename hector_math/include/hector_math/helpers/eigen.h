//
// Created by stefan on 26.11.21.
//

#ifndef HECTOR_MATH_EIGEN_H
#define HECTOR_MATH_EIGEN_H

#include <Eigen/Core>

namespace hector_math
{
namespace eigen
{

template<class ArgType>
struct array_passthrough_helper {
  using ArrayType =
      Eigen::Array<typename ArgType::Scalar, ArgType::RowsAtCompileTime, ArgType::ColsAtCompileTime,
                   Eigen::ColMajor, ArgType::MaxRowsAtCompileTime, ArgType::MaxColsAtCompileTime>;
};

namespace flip_ops
{
enum FlipOp { Rows = 0x1, Columns = 0x2, Both = 0x3 };
}
using flip_ops::FlipOp;

template<typename MatrixType>
class wrap_with_constant_functor;

/*!
 * Wraps the given array with a constant value.
 * Example:
 * @verbatim
 *   1 2
 *   3 4
 * @endverbatim
 * wrapped to rows = 4, cols = 5, row_offset = 1, col_offset = 2 and value = 0 results in
 * @verbatim
 *   0 0 0 0 0
 *   0 0 1 2 0
 *   0 0 3 4 0
 *   0 0 0 0 0
 * @endverbatim
 *
 * @return An array of size rows, cols where every value outside of the by the offset shifted mat is
 *   the passed constant value.
 */
template<typename ArgType>
Eigen::CwiseNullaryOp<wrap_with_constant_functor<ArgType>,
                      typename array_passthrough_helper<ArgType>::ArrayType>
wrapWithConstant( const Eigen::ArrayBase<ArgType> &mat, typename ArgType::Scalar value,
                  Eigen::Index rows, Eigen::Index cols, Eigen::Index row_offset = 0,
                  Eigen::Index column_offset = 0 );

template<typename MatrixType>
class shift_functor;

/*!
 * Shifts the given Eigen Array by the given row and column values.
 * I.e. the access to arr(x, y) is mapped to arr(x+row_shift, y+column_shift).
 * E.g. the array:
 * @verbatim
 *   1 2 3
 *   4 5 6
 *   7 8 9
 * @endverbatim
 * shifted by row_shift=2 and column_shift=1 would result in
 * @verbatim
 *   8 9 7
 *   2 3 1
 *   5 6 4
 * @endverbatim
 *
 * @param row_shift The number of rows that are added to the accessed location.
 *   In essence, this shifts the rows upwards by the given value.
 * @param column_shift The number of columns that are added to the accessed location.
 *   In essence, this shifts the rows to the left by the given value.
 * @return The shifted array.
 */
template<typename ArgType>
Eigen::CwiseNullaryOp<shift_functor<ArgType>, typename array_passthrough_helper<ArgType>::ArrayType>
shift( const Eigen::ArrayBase<ArgType> &mat, Eigen::Index row_shift, Eigen::Index column_shift );

template<typename ArgType, FlipOp FLIP_OP>
class flip_functor;

/*!
 * Flips the given Eigen Array in the specified axis (default: Both).
 * Example:
 * @verbatim
 *   1 2 3
 *   4 5 6
 *   7 8 9
 * @endverbatim
 * flipped with flip_op = flip_ops::Rows results in
 * @verbatim
 *   7 8 9
 *   4 5 6
 *   1 2 3
 * @endverbatim
 * flipped with flip_op = flip_ops::Both instead would result in
 * @verbatim
 *   9 8 7
 *   6 5 4
 *   3 2 1
 * @endverbatim
 *
 * @return The flipped array.
 */
template<typename ArgType>
Eigen::CwiseNullaryOp<flip_functor<ArgType, flip_ops::Both>,
                      typename array_passthrough_helper<ArgType>::ArrayType>
flip( const Eigen::ArrayBase<ArgType> &mat );

// ==============================================
//                 IMPLEMENTATION
// ==============================================

template<typename MatrixType>
class wrap_with_constant_functor
{
public:
  wrap_with_constant_functor( const MatrixType &mat, typename MatrixType::Scalar value,
                              Eigen::Index row_offset = 0, Eigen::Index column_offset = 0 )
      : mat_( mat ), value_( value ), row_offset_( row_offset ), column_offset_( column_offset ),
        mat_rows_( mat.rows() ), mat_cols_( mat.cols() )
  {
  }

  typename MatrixType::Scalar operator()( Eigen::Index row, Eigen::Index col ) const
  {
    if ( row < row_offset_ || col < column_offset_ )
      return value_;
    row -= row_offset_;
    col -= column_offset_;
    if ( row >= mat_rows_ || col >= mat_cols_ )
      return value_;
    return mat_( row, col );
  }

private:
  const typename MatrixType::Nested mat_;
  const typename MatrixType::Scalar value_;
  const Eigen::Index row_offset_;
  const Eigen::Index column_offset_;
  const Eigen::Index mat_rows_;
  const Eigen::Index mat_cols_;
};

template<typename ArgType>
Eigen::CwiseNullaryOp<wrap_with_constant_functor<ArgType>,
                      typename array_passthrough_helper<ArgType>::ArrayType>
wrapWithConstant( const Eigen::ArrayBase<ArgType> &mat, typename ArgType::Scalar value,
                  Eigen::Index rows, Eigen::Index cols, Eigen::Index row_offset,
                  Eigen::Index column_offset )
{
  typedef typename array_passthrough_helper<ArgType>::ArrayType ArrayType;
  return ArrayType::NullaryExpr(
      rows, cols,
      wrap_with_constant_functor<ArgType>( mat.derived(), value, row_offset, column_offset ) );
}

template<typename MatrixType>
class shift_functor
{
public:
  shift_functor( const MatrixType &mat, Eigen::Index row_shift, Eigen::Index column_shift )
      : m_mat( mat ), rows_( mat.rows() ), cols_( mat.cols() )
  {
    if ( row_shift <= -rows_ || row_shift > rows_ ) {
      row_shift = row_shift % rows_;
    }
    if ( row_shift > 0 )
      row_shift -= rows_;
    row_shift_ = row_shift;
    if ( column_shift <= -cols_ || column_shift > cols_ ) {
      column_shift = column_shift % cols_;
    }
    if ( column_shift > 0 )
      column_shift -= cols_;
    column_shift_ = column_shift;
  }

  typename MatrixType::Scalar operator()( Eigen::Index row, Eigen::Index col ) const
  {
    row += row_shift_;
    col += column_shift_;
    if ( row < 0 )
      row += rows_;
    if ( col < 0 )
      col += cols_;
    return m_mat( row, col );
  }

private:
  const typename MatrixType::Nested m_mat;
  Eigen::Index row_shift_;
  Eigen::Index column_shift_;
  const Eigen::Index rows_;
  const Eigen::Index cols_;
};

template<typename ArgType>
Eigen::CwiseNullaryOp<shift_functor<ArgType>, typename array_passthrough_helper<ArgType>::ArrayType>
shift( const Eigen::ArrayBase<ArgType> &mat, Eigen::Index row_shift, Eigen::Index column_shift )
{
  typedef typename array_passthrough_helper<ArgType>::ArrayType ArrayType;
  return ArrayType::NullaryExpr( mat.rows(), mat.cols(),
                                 shift_functor<ArgType>( mat.derived(), row_shift, column_shift ) );
}

template<typename ArgType, FlipOp FLIP_OP>
class flip_functor
{
public:
  explicit flip_functor( const ArgType &mat )
      : m_mat( mat ), row_max_( mat.rows() - 1 ), col_max_( mat.cols() - 1 )
  {
  }

  typename ArgType::Scalar operator()( Eigen::Index row, Eigen::Index col ) const
  {
    return m_mat( ( FLIP_OP & flip_ops::Rows ) == flip_ops::Rows ? row_max_ - row : row,
                  ( FLIP_OP & flip_ops::Columns ) == flip_ops::Columns ? col_max_ - col : col );
  }

private:
  const typename ArgType::Nested m_mat;
  const Eigen::Index row_max_;
  const Eigen::Index col_max_;
};

template<typename ArgType>
Eigen::CwiseNullaryOp<flip_functor<ArgType, flip_ops::Both>,
                      typename array_passthrough_helper<ArgType>::ArrayType>
flip( const Eigen::ArrayBase<ArgType> &mat )
{
  typedef typename array_passthrough_helper<ArgType>::ArrayType ArrayType;
  return ArrayType::NullaryExpr( mat.rows(), mat.cols(),
                                 flip_functor<ArgType, flip_ops::Both>( mat.derived() ) );
}
} // namespace eigen
} // namespace hector_math

#endif // HECTOR_MATH_EIGEN_H
