
#include <hector_math/iterators/circle_iterator.h>
#include <hector_math/iterators/polygon_iterator.h>
#include "iterator_test_input.h"

#include "eigen_tests.h"
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <fstream>
#include <ros/package.h>

using namespace hector_math;

template<typename Scalar>
std::string jsonVector(const GridMap<Scalar> map, int offset){
    std::string s= "[";
    for( int i=0; i<map.rows(); i++){
        for(int j=0;j<map.cols();j++){
            if ( map(i,j) > 0 ){
                s +=  "[" + std::to_string(i-offset) + ", " + std::to_string(j-offset)+"], ";
            }
        }
    }
    s = s.substr(0,s.size()-2) + "],\n";
    return s;
}
// writes JSON File for polygon Test cases, can be visualized with showPolygon.py
// visualizes the polygon, the groundtruth and the points found using the polygon iterator
template<typename Scalar>
void writeReportToFile(const GridMap<Scalar> iteratedMap,const GridMap<Scalar> realMap,hector_math::Polygon<Scalar> polygon,
                       Eigen::Index row_min, Eigen::Index row_max, Eigen::Index col_min, Eigen::Index col_max,int offset, const std::string &name){
    std::string package_path = ros::package::getPath( ROS_PACKAGE_NAME );
    std::string path = package_path + "/test/tmp/" + name;
    std::ofstream output( path, std::ios_base::out );
    if ( !output.is_open())
    {
        std::cerr << "Couldn't open file to dump report!" << std::endl;
        return;
    }
    output<<"{\n \"limits\": [";
    output<<row_min<<", "<<row_max<<", "<<col_min<<", "<<col_max<<"],"<<std::endl;
    output<<"\"iterated positions\":";
    output<<jsonVector(iteratedMap,offset);
    output<<"\"real positions\":";
    output<<jsonVector(realMap,offset);
    output<<"\"corners\": [";
    for(int i=0; i< polygon.cols();i++){
        output<<"["<< polygon(0,i)<< ", "<<polygon(1,i);
        if (i != polygon.cols()-1){
            output <<"],";
        }else{
            output<<"]]"<<std::endl<<"}";
        }
    }
    output.flush();
    output.close();
    std::cout<<"Wrote file"<<path<<std::endl;
}


template<typename Scalar>
class IteratorTest : public testing::Test
{
};


typedef testing::Types<float, double> Implementations;

TYPED_TEST_CASE( IteratorTest, Implementations );

TYPED_TEST( IteratorTest, circleTest ) {
    using Scalar = TypeParam;
    using Vector2 = Vector2<Scalar>;
    //normal case in area x: -4 bis 4 and y: -4 bis 4, center (0,0) and radius 2,
    GridMap<Scalar> realMap(5, 5);
    // @formatter:off
    realMap << 0, 0, 1, 0, 0,
            0, 1, 1, 1, 0,
            1, 1, 1, 1, 0,
            0, 1, 1, 1, 0,
            0, 0, 0, 0, 0;
    // @formatter:on
    GridMap<Scalar> iteratedMap(5, 5); iteratedMap.setZero();
    iterateCircle<Scalar>(Vector2(Scalar(0.49), Scalar(0.49)), 2, Eigen::Index(-4), Eigen::Index(4), Eigen::Index(-4),
                          Eigen::Index(4), [&iteratedMap](Eigen::Index x, Eigen::Index y) {
                iteratedMap(x + 2, y + 2) += 1;
            });
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(realMap, iteratedMap, 1E-6));

    // case 2, equal to one but center at (0,0)
    // @formatter:off
    realMap << 0, 1, 1, 0, 0,
            1, 1, 1, 1, 0,
            1, 1, 1, 1, 0,
            0, 1, 1, 0, 0,
            0, 0, 0, 0, 0;
    // @formatter:on
    iteratedMap.setZero();
    iterateCircle<Scalar>(Vector2(Scalar(0),Scalar(0)),2,Eigen::Index(-4),Eigen::Index(4),Eigen::Index(-4),Eigen::Index(4) ,[ &iteratedMap ]( Eigen::Index x, Eigen::Index y )
    {
        iteratedMap(x + 2, y + 2) += 1;
    } );
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(realMap, iteratedMap, 1E-6));

    //case 3 restrict max_row to be 1
    realMap = GridMap<Scalar>(4, 5);
    iteratedMap = GridMap<Scalar>(4, 5);iteratedMap.setZero();
    // @formatter:off
    realMap << 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 1.0, 0.0,
            0.0, 1.0, 1.0, 1.0, 1.0,
            0.0, 1.0, 1.0, 1.0, 1.0;
    // @formatter:on
    iterateCircle<Scalar>(Vector2(Scalar(0),Scalar(0)),2,Eigen::Index(-4),Eigen::Index(1),Eigen::Index(-4),Eigen::Index(4) ,[ &iteratedMap ]( Eigen::Index x, Eigen::Index y )
    {
        iteratedMap(x + 3, y + 3) += 1;
    } );
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(realMap, iteratedMap, 1E-6));

    //case 4 restrict min_row to be 1
    realMap = GridMap<Scalar>(3, 4);
    iteratedMap = GridMap<Scalar>(3, 4);iteratedMap.setZero();
    // @formatter:off
    realMap << 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0;
    // @formatter:on
    iterateCircle<Scalar>(Vector2(Scalar(0),Scalar(0)),2,Eigen::Index(1),Eigen::Index(4),Eigen::Index(-4),Eigen::Index(4) ,[ &iteratedMap ]( Eigen::Index x, Eigen::Index y )
    {
        iteratedMap(x + 0, y + 2) += 1;
    } );
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(realMap, iteratedMap, 1E-6));
    //case 5 restrict min_column to be -1 and max_column to be 1
    realMap = GridMap<Scalar>(6, 4);
    iteratedMap = GridMap<Scalar>(6, 4); iteratedMap.setZero();
    // @formatter:off
    realMap << 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 1.0, 0.0,
            0.0, 1.0, 1.0, 0.0,
            0.0, 1.0, 1.0, 0.0,
            0.0, 1.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0;
    // @formatter:on
    iterateCircle<Scalar>(Vector2(Scalar(0),Scalar(0)),2,Eigen::Index(-4),Eigen::Index(4),Eigen::Index(-1),Eigen::Index(1) ,[ &iteratedMap ]( Eigen::Index x, Eigen::Index y )
    {
        iteratedMap(x  + 3, y + 2) += 1;
    } );
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(realMap, iteratedMap, 1E-6));

    //case 6 no points due to colum/index restrictions
    realMap = GridMap<Scalar>(6, 4);
    iteratedMap = GridMap<Scalar>(6, 4);
    iteratedMap.setZero(); realMap.setZero();
    iterateCircle<Scalar>(Vector2(Scalar(0),Scalar(0)),2,Eigen::Index(4),Eigen::Index(8),Eigen::Index(-8),Eigen::Index(8) ,[ &iteratedMap ]( Eigen::Index x, Eigen::Index y )
    {
        iteratedMap(x  + 3, y + 3) += 1;
    } );
    iterateCircle<Scalar>(Vector2(Scalar(0),Scalar(0)),2,Eigen::Index(-8),Eigen::Index(8),Eigen::Index(4),Eigen::Index(8) ,[ &iteratedMap ]( Eigen::Index x, Eigen::Index y )
    {
        iteratedMap(x  + 3, y + 3) += 1;
    } );
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(realMap, iteratedMap, 1E-6));
    // case 7, using different function overload
    realMap = GridMap<Scalar>(4, 4);
    iteratedMap = GridMap<Scalar>(4, 4);iteratedMap.setZero();
    // @formatter:off
    realMap << 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 1.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0;
    // @formatter:on
    iterateCircle<Scalar>(Vector2(Scalar(0),Scalar(0)),2,Eigen::Index(4),Eigen::Index(4) ,[ &iteratedMap ]( Eigen::Index x, Eigen::Index y )
    {
        iteratedMap(x  + 1, y + 1) += 1;
    } );
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(realMap, iteratedMap, 1E-6));
}

TYPED_TEST( IteratorTest, polygonTest ) {
    using Scalar = TypeParam;
    using Vector2 = Vector2<Scalar>;
    std::vector<Vector2> position;
    int offset = 5;
    Polygon<Scalar> polygon = createPolygon<Scalar>(0);
    //normal case in area x: -4 bis 4 and y: -4 bis 4, center (0,0) and radius 2,
    Eigen::Index row_min = -6;
    Eigen::Index row_max = 6;
    Eigen::Index col_min = -6;
    Eigen::Index col_max = 6;
    GridMap<Scalar> realMap(10, 10);
    GridMap<Scalar> iteratedMap(10, 10);
    iteratedMap.setZero();
    // @formatter:off
    realMap << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 1.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 1.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // @formatter:on
    iteratePolygon<Scalar>(polygon,row_min,row_max,col_min,col_max ,[ &iteratedMap,offset ]( Eigen::Index x, Eigen::Index y )
    {
        iteratedMap(x  + offset, y + offset) += 1;
    } );
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(realMap, iteratedMap, 1E-6));
    writeReportToFile(iteratedMap, realMap,polygon,row_min,row_max,col_min,col_max,offset,"TestCasePolygonRandom.txt");
    // case Z
    polygon = createPolygon<Scalar>(1);
    iteratedMap.setZero();
    // @formatter:off
    realMap << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // @formatter:on
    iteratePolygon<Scalar>(polygon,row_min,row_max,col_min,col_max ,[ &iteratedMap,offset ]( Eigen::Index x, Eigen::Index y )
    {
        iteratedMap(x  + offset, y + offset) += 1;
    } );
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(realMap, iteratedMap, 1E-6));
    writeReportToFile(iteratedMap, realMap,polygon,row_min,row_max,col_min,col_max,offset,"TestCasePolygonZShape.txt");
    // circle approximation
    polygon = createPolygon<Scalar>(2);
    iteratedMap.setZero();
    // @formatter:off
    realMap << 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0,
            0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0,
            0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0,
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
            0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0,
            0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0,
            0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    // @formatter:on
    iteratePolygon<Scalar>(polygon,row_min,row_max,col_min,col_max ,[ &iteratedMap,offset ]( Eigen::Index x, Eigen::Index y )
    {
        iteratedMap(x  + offset, y + offset) += 1;
    } );
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(realMap, iteratedMap, 1E-6));
    writeReportToFile(iteratedMap, realMap,polygon,row_min,row_max,col_min,col_max,offset,"TestCaseCircleShape.txt");

    // u - shape
    polygon = createPolygon<Scalar>(3);
    iteratedMap.setZero();
    // @formatter:off
    realMap << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
    // @formatter:on
    iteratePolygon<Scalar>(polygon,row_min,row_max,col_min,col_max ,[ &iteratedMap,offset ]( Eigen::Index x, Eigen::Index y )
    {
        std::cout<<"x "<<x<<" y "<<y<<std::endl;
        iteratedMap(x  + offset, y + offset) += 1;
    } );
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(realMap, iteratedMap, 1E-6));
    writeReportToFile(iteratedMap, realMap,polygon,row_min,row_max,col_min,col_max,offset,"TestCaseUShape.txt");

    // circle approximation with limited indexes
    row_min =-4;
    row_max = 2;
    col_min = -3;
    col_max = 1;
    polygon = createPolygon<Scalar>(2);
    iteratedMap.setZero();
    // @formatter:off
    realMap << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // @formatter:on
    iteratePolygon<Scalar>(polygon,row_min,row_max,col_min,col_max ,[ &iteratedMap,offset ]( Eigen::Index x, Eigen::Index y )
    {
        iteratedMap(x  + offset, y + offset) += 1;
    } );
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(realMap, iteratedMap, 1E-6));
    writeReportToFile(iteratedMap, realMap,polygon,row_min,row_max,col_min,col_max,offset,"TestCaseCircleShapeLimitedIndexes.txt");

    // u - shape limited index
    polygon = createPolygon<Scalar>(3);
    iteratedMap.setZero();
    // @formatter:off
    realMap << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // @formatter:on
    iteratePolygon<Scalar>(polygon,row_min,row_max,col_min,col_max ,[ &iteratedMap,offset ]( Eigen::Index x, Eigen::Index y )
    {
        iteratedMap(x  + offset, y + offset) += 1;
    } );
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(realMap, iteratedMap, 1E-6));
    writeReportToFile(iteratedMap, realMap,polygon,row_min,row_max,col_min,col_max,offset,"TestCaseUShapeLimitedIndexes.txt");

}

int main( int argc, char **argv )
{
    ros::init( argc, argv, "test_hector_iterators" );
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
