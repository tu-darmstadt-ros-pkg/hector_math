=============
Eigen Helpers
=============

These three methods allow altering eigen vectors and matrices efficiently. The matrices can be
shifted, flipped and wrapped with a constant value.
The functions can be used with matrices whose size is determined at compile or runtime.
The first function, `wrapWithConstant` extends the matrix to the given size and all new
elements are assigned the same value.
The function `shift` moves every element according to the given number of rows and columns.
Lastly, `flip` mirrors the array along the specified axis.

API
---

.. doxygenfunction:: hector_math::eigen::wrapWithConstant

.. doxygenfunction:: hector_math::eigen::shift

.. doxygenenum:: hector_math::eigen::flip_ops::FlipOp

.. doxygenfunction:: hector_math::eigen::flip( const Eigen::ArrayBase<ArgType> &mat )

.. doxygenfunction:: hector_math::eigen::flip( const Eigen::ArrayBase<ArgType> &mat, FlipOp flip_op );