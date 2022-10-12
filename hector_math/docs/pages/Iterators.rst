=========
Iterators
=========

These functions allow calling a functor on all cells in a given shape. The functions iterate over
all cells whose center lies in the shape and calls the given functor with the cell indexes as arguments.
Three different shapes are available, namely rectangle, polygon and circle.

API
---

.. doxygenfunction:: hector_math::iterateCircle( const Vector2<Scalar> &center, Scalar radius, Eigen::Index row_min,Eigen::Index row_max, Eigen::Index col_min, Eigen::Index col_max,Functor functor )

.. doxygenfunction:: hector_math::iteratePolygon( const Polygon<Scalar> &polygon, Eigen::Index row_min, Eigen::Index row_max,Eigen::Index col_min, Eigen::Index col_max, Functor functor )

.. doxygenfunction:: hector_math::iterateRectangle( const Vector2<Scalar> &a, const Vector2<Scalar> &b, const Vector2<Scalar> &c,Eigen::Index row_min, Eigen::Index row_max, Eigen::Index col_min,Eigen::Index col_max, Functor functor )


