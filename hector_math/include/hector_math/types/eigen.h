// Copyright (c) 2023 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_TYPES_EIGEN_H
#define HECTOR_MATH_TYPES_EIGEN_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

namespace hector_math
{

template<typename Scalar>
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
using Matrix3f = Matrix3<float>;
using Matrix3d = Matrix3<double>;

template<typename Scalar>
using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
using Vector2f = Vector2<float>;
using Vector2d = Vector2<double>;
template<typename Scalar>
using Vector2List = std::vector<Vector2<Scalar>, Eigen::aligned_allocator<Vector2<Scalar>>>;
using Vector2fList = Vector2List<float>;
using Vector2dList = Vector2List<double>;

template<typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
using Vector3f = Vector3<float>;
using Vector3d = Vector3<double>;
template<typename Scalar>
using Vector3List = std::vector<Vector3<Scalar>>;
using Vector3fList = Vector3List<float>;
using Vector3dList = Vector3List<double>;

template<typename Scalar>
using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
using Vector4f = Vector4<float>;
using Vector4d = Vector4<double>;
template<typename Scalar>
using Vector4List = std::vector<Vector4<Scalar>, Eigen::aligned_allocator<Vector4<Scalar>>>;
using Vector4fList = Vector4List<float>;
using Vector4dList = Vector4List<double>;

template<typename Scalar>
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using VectorXf = VectorX<float>;
using VectorXd = VectorX<double>;
template<typename Scalar>
using VectorXList = std::vector<VectorX<Scalar>>;
using VectorXfList = VectorXList<float>;
using VectorXdList = VectorXList<double>;

template<typename Scalar>
using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
using Isometry3f = Isometry3<float>;
using Isometry3d = Isometry3<double>;

template<typename Scalar>
using Isometry2 = Eigen::Transform<Scalar, 2, Eigen::Isometry>;
using Isometry2f = Isometry2<float>;
using Isometry2d = Isometry2<double>;

template<typename Scalar>
using Point = Eigen::Array<Scalar, 2, 1>;
using Pointf = Point<float>;
using Pointd = Point<double>;
template<typename Scalar>
using PointList = std::vector<Point<Scalar>, Eigen::aligned_allocator<Point<Scalar>>>;
using PointfList = PointList<float>;
using PointdList = PointList<double>;

template<typename Scalar>
using Polygon = Eigen::Array<Scalar, 2, Eigen::Dynamic>;
using Polygonf = Polygon<float>;
using Polygond = Polygon<double>;

template<typename Scalar>
using GridMap = Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
using GridMapf = GridMap<float>;
using GridMapd = GridMap<double>;

struct Index2D {
  explicit Index2D( Eigen::Index row = 0, Eigen::Index col = 0 ) : row( row ), col( col ) { }

  Eigen::Index row;
  Eigen::Index col;

  friend bool operator==( const Index2D &lhs, const Index2D &rhs )
  {
    return lhs.row == rhs.row && lhs.col == rhs.col;
  }

  friend bool operator!=( const Index2D &lhs, const Index2D &rhs ) { return !( lhs == rhs ); }

  Eigen::Index manhattanDistance( const Index2D &other ) const
  {
    return std::abs( row - other.row ) + std::abs( col - other.col );
  }
};

struct BlockIndices {
  Eigen::Index x0 = 0;
  Eigen::Index y0 = 0;
  Eigen::Index rows = 0;
  Eigen::Index cols = 0;

  BlockIndices() = default;
  BlockIndices( Eigen::Index x0, Eigen::Index y0, Eigen::Index rows, Eigen::Index cols )
      : x0( x0 ), y0( y0 ), rows( rows ), cols( cols )
  {
  }

  static BlockIndices Empty() { return { 0, 0, 0, 0 }; }

  [[nodiscard]] bool empty() const { return rows == 0 || cols == 0; }

  friend bool operator==( const BlockIndices &a, const BlockIndices &b )
  {
    if ( a.empty() )
      return b.empty();
    return a.x0 == b.x0 && a.y0 == b.y0 && a.rows == b.rows && a.cols == b.cols;
  }

  friend bool operator!=( const BlockIndices &a, const BlockIndices &b ) { return !( a == b ); }

  bool contains( Eigen::Index row, Eigen::Index col ) const
  {
    return row >= x0 && row < x0 + rows && col >= y0 && col < y0 + cols;
  }

  [[nodiscard]] BlockIndices include( const BlockIndices &other ) const
  {
    if ( empty() )
      return other;
    if ( other.empty() )
      return *this;
    BlockIndices result = *this;
    result.x0 = std::min( x0, other.x0 );
    result.y0 = std::min( y0, other.y0 );
    result.rows = std::max( x0 + rows, other.x0 + other.rows ) - result.x0;
    result.cols = std::max( y0 + cols, other.y0 + other.cols ) - result.y0;
    return result;
  }

  BlockIndices &includeInPlace( const BlockIndices &other )
  {
    *this = include( other );
    return *this;
  }

  [[nodiscard]] BlockIndices include( Eigen::Index row, Eigen::Index col ) const
  {
    if ( empty() )
      return { row, col, 1, 1 };
    BlockIndices result = *this;
    result.x0 = std::min( x0, row );
    result.y0 = std::min( y0, col );
    result.rows = std::max( x0 + rows, row + 1 ) - result.x0;
    result.cols = std::max( y0 + cols, col + 1 ) - result.y0;
    return result;
  }

  BlockIndices &includeInPlace( Eigen::Index row, Eigen::Index col )
  {
    *this = include( row, col );
    return *this;
  }

  [[nodiscard]] BlockIndices scale( double scale )
  {
    BlockIndices result = *this;
    result.x0 = static_cast<Eigen::Index>( std::floor( x0 * scale ) );
    result.y0 = static_cast<Eigen::Index>( std::floor( y0 * scale ) );
    result.rows = static_cast<Eigen::Index>( std::ceil( ( x0 + rows ) * scale ) - result.x0 );
    result.cols = static_cast<Eigen::Index>( std::ceil( ( y0 + cols ) * scale ) - result.y0 );
    return result;
  }

  [[nodiscard]] BlockIndices intersect( const BlockIndices &other ) const
  {
    if ( empty() || other.empty() )
      return Empty();
    BlockIndices result;
    result.x0 = std::max( x0, other.x0 );
    result.y0 = std::max( y0, other.y0 );
    const Eigen::Index x1 = std::min( x0 + rows, other.x0 + other.rows );
    const Eigen::Index y1 = std::min( y0 + cols, other.y0 + other.cols );
    if ( x1 <= result.x0 || y1 <= result.y0 )
      return Empty();
    result.rows = x1 - result.x0;
    result.cols = y1 - result.y0;
    return result;
  }

  [[nodiscard]] BlockIndices intersect( Eigen::Index x0, Eigen::Index y0, Eigen::Index rows,
                                        Eigen::Index cols ) const
  {
    return intersect( { x0, y0, rows, cols } );
  }
};
} // namespace hector_math

template<>
struct std::hash<hector_math::Index2D> {
  std::size_t operator()( const hector_math::Index2D &index ) const noexcept
  {
    // Yes, technically this does not cover the whole range of Eigen::Index but in practice
    // an index will not exceed 2^32-1 in either dimension, so this is sufficient.
    assert( std::abs( index.row ) < ( 1L << 32 ) && std::abs( index.col ) < ( 1L << 32 ) &&
            "Index2D values are assumed to be within the range of a 32-bit signed integer." );
    return std::hash<long>()( ( index.row << 32L ) + index.col );
  }
};

#endif // HECTOR_MATH_TYPES_EIGEN_H
