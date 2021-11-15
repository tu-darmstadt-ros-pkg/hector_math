#ifndef HECTOR_MATH_ITERATOR_TEST_INPUT_H
#define HECTOR_MATH_ITERATOR_TEST_INPUT_H
using namespace hector_math;
///////////////// Polygon /////////////////////
template<typename Scalar>
hector_math::Polygon<Scalar> createPolygon(int which)
{
    if (which == 0) {
        hector_math::Polygon<Scalar> result(2, 15);
        result.col(0) << 0, 4.9;
        result.col(1) << 3.5, 4.9;
        result.col(2) << 4.3333, 3.3333;
        result.col(3) << 4, 2.3333;
        result.col(4) << 0, 2;
        result.col(5) << 0, 0.95;
        result.col(6) << 4, 0;
        result.col(7) << 0, 0;
        result.col(8) << -0.8, -1.0;
        result.col(9) << -0, -2.0;
        result.col(10) << -4.01, -0.99;
        result.col(11) << -4, 1;
        result.col(12) << -3, 4;
        result.col(13) << -2, 4;
        result.col(14) << -1.0, 2.5;
        return result;
    }else if (which == 1){
        // Z structure
        hector_math::Polygon<Scalar> result(2, 8);
        result.col(0) << 1.66666, 4;
        result.col(1) << -5,4;
        result.col(2) << -5,3;
        result.col(3) << 0,3;
        result.col(4) << -4.66666, -4;
        result.col(5) << 2, -4;
        result.col(6) << 2,-3;
        result.col(7) << -3,-3;
        return result;
    }else if (which == 2) {
        // circle structure
        hector_math::Polygon<Scalar> result(2, 8);
        result.col(0) << 0, 5;
        result.col(1) << -3.6, 3.5;
        result.col(2) << -5, 0;
        result.col(3) << -3.6, -3.5;
        result.col(4) << 0, -5;
        result.col(5) << 3.6, -3.5;
        result.col(6) << 5, 0;
        result.col(7) << 3.6, 3.5;
        return result;
    }else if (which == 3) {
        // u structure
        hector_math::Polygon<Scalar> result(2, 11);
        result.col(0) << -3.5, 4.2;
        result.col(1) << -2, 4.2;
        result.col(2) << -2, -0.5;
        result.col(3) << 1, -0.5;
        result.col(4) << 1, 4.2;
        result.col(5) << 3.5, 4.2;
        result.col(6) << 5.5, -0.5;
        result.col(7) << 4.5, -2;
        result.col(8) << 2.4, 3.5;
        result.col(9) << 2.4, -2;
        result.col(10) << -3.5, -2;
        return result;
    }
    return hector_math::Polygon<Scalar>(2, 0);
}
///////////////// groundtruth /////////////////////
//template<typename Scalar>
//std::vector<Vector<Scalar>> getGroundtruth(int which){

//}

///////////////// Polygon /////////////////////

#endif //HECTOR_MATH_ITERATOR_TEST_INPUT_H