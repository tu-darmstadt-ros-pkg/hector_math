=============
Eigen Helpers
=============

These three methods allow altering eigen vectors and matrices efficiently. The matrices can be
shifted, flipped and wrapped with a constant value.
The functions can be used with matrices whose size is determined at compile or runtime.
The first function, `wrapWithConstant` extends the matrix to the given size and all new
elements are assigned the same value. An example can be seen below in the definition of
`wrapWithConstant() <Wrap With Constant_>`_.
The function `shift` moves every element according to the given number of rows and columns.
The `shift() <Shift_>`_ definition down below provides a simple example.
Lastly, `flip` mirrors the array along the specified axis. The axis to flip can be specified
by selecting a member of the enum `FlipOP`. Examples can be found in the `flip() <Flip_>`_ definition.

API
---
Wrap With Constant
******************
.. doxygenfunction:: hector_math::eigen::wrapWithConstant

Shift
*****
.. doxygenfunction:: hector_math::eigen::shift
Flip
****
.. doxygenenum:: hector_math::eigen::flip_ops::FlipOp

.. doxygenfunction:: hector_math::eigen::flip( const Eigen::ArrayBase<ArgType> &mat )

.. doxygenfunction:: hector_math::eigen::flip( const Eigen::ArrayBase<ArgType> &mat, FlipOp flip_op );