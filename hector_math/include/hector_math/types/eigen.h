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
template<typename Scalar>
using PointList = std::vector<Point<Scalar>, Eigen::aligned_allocator<Point<Scalar>>>;

template<typename Scalar>
using Polygon = Eigen::Array<Scalar, 2, Eigen::Dynamic>;

template<typename Scalar>
using GridMap = Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

struct BlockIndices {
  Eigen::Index x0;
  Eigen::Index y0;
  Eigen::Index rows;
  Eigen::Index cols;
};
} // namespace hector_math

#endif // HECTOR_MATH_TYPES_EIGEN_H
