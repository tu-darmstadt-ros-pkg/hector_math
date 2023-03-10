// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_EIGEN_TESTS_H
#define HECTOR_EIGEN_TESTS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#define EIGEN_MATRIX_EQUAL( A, B ) DenseBaseEqual( A, #A, B, #B )
#define EIGEN_ARRAY_EQUAL( A, B ) DenseBaseEqual( A, #A, B, #B )
#define EIGEN_QUATERNIONS_EQUAL( A, B ) QuaternionBaseEqual( A, #A, B, #B )
#define EIGEN_VECTOR_EQUAL( A, B ) DenseBaseEqual( A, #A, B, #B )

#define EIGEN_MATRIX_NEAR( A, B, precision ) DenseBaseNear( A, #A, B, #B, precision )
#define EIGEN_ARRAY_NEAR( A, B, precision ) DenseBaseNear( A, #A, B, #B, precision )
#define EIGEN_QUATERNIONS_NEAR( A, B, precision ) QuaternionBaseNear( A, #A, B, #B, precision )
#define EIGEN_VECTOR_NEAR( A, B, precision ) DenseBaseNear( A, #A, B, #B, precision )

#define EIGEN_MATRIX_SAME_FINITE( A, B ) DenseBaseSameFinite( A, #A, B, #B )
#define EIGEN_ARRAY_SAME_FINITE( A, B ) DenseBaseSameFinite( A, #A, B, #B )

template<typename DerivedA, typename DerivedB>
::testing::AssertionResult
DenseBaseEqual( const Eigen::DenseBase<DerivedA> &lh, const std::string &lh_name,
                const Eigen::DenseBase<DerivedB> &rh, const std::string &rh_name )
{
  if ( lh.rows() != rh.rows() || lh.cols() != rh.cols() ) {
    return ::testing::AssertionFailure()
           << "Matrix sizes do not match!"
           << " (" << lh_name << ") " << lh.rows() << "x" << lh.cols() << " vs " << rh.rows() << "x"
           << rh.cols() << " (" << rh_name << ").";
  }
  for ( Eigen::Index col = 0; col < lh.cols(); ++col ) {
    for ( Eigen::Index row = 0; row < lh.rows(); ++row ) {
      if ( lh( row, col ) == rh( row, col ) )
        continue;
      if ( std::isnan( lh( row, col ) ) && std::isnan( rh( row, col ) ) )
        continue;

      return ::testing::AssertionFailure()
             << "Matrices not equal!" << std::endl
             << lh_name << " at " << row << ", " << col << ": " << lh( row, col ) << std::endl
             << rh_name << " at " << row << ", " << col << ": " << rh( row, col ) << std::endl
             << lh_name << ":" << std::endl
             << lh << std::endl
             << rh_name << ":" << std::endl
             << rh;
    }
  }
  return ::testing::AssertionSuccess();
}

template<typename DerivedA, typename DerivedB>
::testing::AssertionResult
DenseBaseSameFinite( const Eigen::DenseBase<DerivedA> &lh, const std::string &lh_name,
                     const Eigen::DenseBase<DerivedB> &rh, const std::string &rh_name )
{
  if ( lh.rows() != rh.rows() || lh.cols() != rh.cols() ) {
    return ::testing::AssertionFailure()
           << "Matrix sizes do not match!"
           << " (" << lh_name << ") " << lh.rows() << "x" << lh.cols() << " vs " << rh.rows() << "x"
           << rh.cols() << " (" << rh_name << ").";
  }
  for ( Eigen::Index col = 0; col < lh.cols(); ++col ) {
    for ( Eigen::Index row = 0; row < lh.rows(); ++row ) {
      if ( std::isfinite( lh( row, col ) ) == std::isfinite( rh( row, col ) ) )
        continue;

      return ::testing::AssertionFailure()
             << "Matrices finite fields not equal!" << std::endl
             << lh_name << " at " << row << ", " << col << ": " << lh( row, col ) << std::endl
             << rh_name << " at " << row << ", " << col << ": " << rh( row, col ) << std::endl
             << lh_name << ":" << std::endl
             << lh << std::endl
             << rh_name << ":" << std::endl
             << rh;
    }
  }
  return ::testing::AssertionSuccess();
}

template<typename DerivedA, typename DerivedB>
::testing::AssertionResult
DenseBaseNear( const Eigen::DenseBase<DerivedA> &lh, const std::string &lh_name,
               const Eigen::DenseBase<DerivedB> &rh, const std::string &rh_name,
               typename std::common_type<typename Eigen::DenseBase<DerivedA>::Scalar,
                                         typename Eigen::DenseBase<DerivedB>::Scalar>::type precision )
{
  if ( lh.rows() != rh.rows() || lh.cols() != rh.cols() ) {
    return ::testing::AssertionFailure()
           << "Matrix sizes do not match!"
           << " (" << lh_name << ") " << lh.rows() << "x" << lh.cols() << " vs " << rh.rows() << "x"
           << rh.cols() << " (" << rh_name << ").";
  }
  for ( Eigen::Index col = 0; col < lh.cols(); ++col ) {
    for ( Eigen::Index row = 0; row < lh.rows(); ++row ) {
      if ( std::abs( lh( row, col ) - rh( row, col ) ) <= precision )
        continue;
      if ( std::isnan( lh( row, col ) ) && std::isnan( rh( row, col ) ) )
        continue;

      auto result = ::testing::AssertionFailure()
                    << "Matrices not equal!" << std::endl
                    << lh_name << " at " << row << ", " << col << ": " << lh( row, col ) << std::endl
                    << rh_name << " at " << row << ", " << col << ": " << rh( row, col ) << std::endl
                    << "Maximum allowed error: " << precision << std::endl
                    << lh_name << ":" << std::endl
                    << lh << std::endl
                    << rh_name << ":" << std::endl
                    << rh;
      return result;
    }
  }
  return ::testing::AssertionSuccess();
}

template<typename DerivedA, typename DerivedB>
::testing::AssertionResult
QuaternionBaseEqual( const Eigen::QuaternionBase<DerivedA> &lh, const std::string &lh_name,
                     const Eigen::QuaternionBase<DerivedB> &rh, const std::string &rh_name )
{
  if ( lh.isApprox( rh ) )
    return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << "Quaternions not equal!" << std::endl
                                       << lh_name << " (w, x, y, z): " << lh.w() << ", " << lh.x()
                                       << ", " << lh.y() << ", " << lh.z() << std::endl
                                       << rh_name << " (w, x, y, z): " << rh.w() << ", " << rh.x()
                                       << ", " << rh.y() << ", " << rh.z();
}

template<typename DerivedA, typename DerivedB, typename Scalar>
::testing::AssertionResult QuaternionBaseNear( const Eigen::QuaternionBase<DerivedA> &lh,
                                               const std::string &lh_name,
                                               const Eigen::QuaternionBase<DerivedB> &rh,
                                               const std::string &rh_name, Scalar precision )
{
  if ( std::abs( lh.angularDistance( rh ) ) < precision )
    return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << "Quaternions not equal!" << std::endl
                                       << lh_name << " (w, x, y, z): " << lh.w() << ", " << lh.x()
                                       << ", " << lh.y() << ", " << lh.z() << std::endl
                                       << rh_name << " (w, x, y, z): " << rh.w() << ", " << rh.x()
                                       << ", " << rh.y() << ", " << rh.z();
}

#endif // HECTOR_EIGEN_TESTS_H
