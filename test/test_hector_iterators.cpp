
#include <hector_math/iterators/circle_iterator.h>
#include <hector_math/iterators/polygon_iterator.h>

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <fstream>
#include <ros/package.h>

using namespace hector_math;

template<typename Scalar>
void writeReportToFile(const std::vector<Vector2<Scalar>> iteratedPositions,const std::vector<Vector2<Scalar>> realPositions,hector_math::Polygon<Scalar> polygon,
                       Eigen::Index row_min, Eigen::Index row_max, Eigen::Index col_min, Eigen::Index col_max, const std::string &name){
    std::string package_path = ros::package::getPath( ROS_PACKAGE_NAME );
    std::string path = package_path + "/test/tmp/" + name;
    std::cerr<<path<<std::endl;
    std::ofstream output( path, std::ios_base::out );
    if ( !output.is_open())
    {
        std::cerr << "Couldn't open file to dump report!" << std::endl;
        return;
    }
    output<<"Limitations"<<std::endl;
    output<<row_min<<", "<<row_max<<std::endl;
    output<<col_min<<", "<<col_max<<std::endl;
    output<<"iterated Positions"<<std::endl;
    for(auto corner :iteratedPositions){
        output<<corner[0]<<", "<<corner[1]<<std::endl;
    }
    output<<"real Positions"<<std::endl;
    for(auto corner :realPositions){
        output<<corner[0]<<", "<<corner[1]<<std::endl;
    }
    output<<"Corners"<<std::endl;
    for(int i=0; i< polygon.cols();i++){
        output<<polygon(0,i)<<", "<<polygon(1,i)<<std::endl;
    }
    output.flush();
    output.close();
    std::cout<<"Wrote file"<<path<<std::endl;
}

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

template<typename Scalar>
bool verifyPositions(const std::vector<Vector2<Scalar>> &encounteredPositions, const std::vector<Vector2<Scalar>> &realPositions,std::string msg){
    bool noErrorsFound = true;
    int a = (int) encounteredPositions.size();
    //ASSERT_EQ(encounteredPositions.size(),realPositions.size())<<"Number of encountered positions and real positions doesn't match";
    for (auto encounteredPosition:encounteredPositions){
        bool exist = false;
        for(auto real_position:realPositions){
            if (encounteredPosition == real_position) exist=true;
        }
        EXPECT_TRUE(exist)<<encounteredPosition[0]<<", "<<encounteredPosition[1]<<" not in real Positions in "<<msg;
        if (!exist) noErrorsFound = false;
    }
    for(auto real_position:realPositions) {
        bool exist = false;
        for (auto encounteredPosition:encounteredPositions){
            //if (pow(encounteredPosition[0]-real_position[0]+0.5,2),pow(encounteredPosition[1]-real_position[1]+0.5,2)<0.3) exist=true;
            if (encounteredPosition == real_position) exist=true;
        }
        EXPECT_TRUE(exist)<<real_position[0]<<", "<<real_position[1]<<" not in encounterd Positions in "<<msg;
        if (!exist) noErrorsFound = false;
    }
    // verify that every entry is only once in encountered positions
    for(auto i = encounteredPositions.begin(); i != encounteredPositions.end(); ++i){
        int count = 0;
        bool not_twice = true;
        for(auto j = i + 1; j != encounteredPositions.end(); ++j){
            if(*i == *j) not_twice= false;
        }
        EXPECT_TRUE(not_twice)<<*i<<"exists twice in the list of encountered Positions while iterating through the polygon/circle in "<<msg;
        if (!not_twice) noErrorsFound = false;
    }
    return noErrorsFound;
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

    std::vector<Vector2> position;
    //normal case in area x: -4 bis 4 and y: -4 bis 4, center (0,0) and radius 2,
    iterateCircle<Scalar>(Vector2(Scalar(0.49),Scalar(0.49)),2,Eigen::Index(-4),Eigen::Index(4),Eigen::Index(-4),Eigen::Index(4) ,[ &position ]( Eigen::Index x, Eigen::Index y )
    {
        position.push_back(Vector2(x,y));
    } );
    std::vector<Vector2> realPositions{Vector2(0,-2),Vector2(-1,-1),Vector2(0,-1),Vector2(1,-1),
                                         Vector2(-2,0),Vector2(-1,0),Vector2(0,0),Vector2(1,0),
                                         Vector2(-1,1),Vector2(0,1),Vector2(1,1)};
    verifyPositions<Scalar>(position,realPositions,"normal case, center at (0.49,0.49");
    // case 2, equal to one but center at (0,0)
    position.clear();
    iterateCircle<Scalar>(Vector2(Scalar(0),Scalar(0)),2,Eigen::Index(-4),Eigen::Index(4),Eigen::Index(-4),Eigen::Index(4) ,[ &position ]( Eigen::Index x, Eigen::Index y )
    {
        position.push_back(Vector2(x,y));
    } );
    realPositions = {Vector2(-1,-2),Vector2(0,-2),
                     Vector2(-2,-1),Vector2(-1,-1),Vector2(0,-1),Vector2(1,-1),
                     Vector2(-2,0),Vector2(-1,0),Vector2(0,0),Vector2(1,0),
                     Vector2(-1,1),Vector2(0,1)};
    verifyPositions<Scalar>(position,realPositions,"normal case center at (0,0)");
    //case 3 restrict max_row to be 1
    position.clear();
    iterateCircle<Scalar>(Vector2(Scalar(0),Scalar(0)),2,Eigen::Index(-4),Eigen::Index(1),Eigen::Index(-4),Eigen::Index(4) ,[ &position ]( Eigen::Index x, Eigen::Index y )
    {
        position.push_back(Vector2(x,y));
    } );
    realPositions = {Vector2(-1,-2),Vector2(0,-2),
                     Vector2(-2,-1),Vector2(-1,-1),Vector2(0,-1),
                     Vector2(-2,0),Vector2(-1,0),Vector2(0,0),
                     Vector2(-1,1),Vector2(0,1)};
    verifyPositions<Scalar>(position,realPositions," case with center at (0,0) and limited max_row");

    //case 4 restrict min_row to be 1
    position.clear();
    iterateCircle<Scalar>(Vector2(Scalar(0),Scalar(0)),2,Eigen::Index(1),Eigen::Index(4),Eigen::Index(-4),Eigen::Index(4) ,[ &position ]( Eigen::Index x, Eigen::Index y )
    {
        position.push_back(Vector2(x,y));
    } );
    realPositions = {Vector2(1,0),Vector2(1,-1)};
    verifyPositions<Scalar>(position,realPositions," case with center at (0,0) and limited min_row");

    //case 5 restrict min_column to be -1 and max_column to be 1
    position.clear();
    iterateCircle<Scalar>(Vector2(Scalar(0),Scalar(0)),2,Eigen::Index(-4),Eigen::Index(4),Eigen::Index(-1),Eigen::Index(1) ,[ &position ]( Eigen::Index x, Eigen::Index y )
    {
        position.push_back(Vector2(x,y));
    } );
    realPositions = {Vector2(-2,-1),Vector2(-1,-1),Vector2(0,-1),Vector2(1,-1),
                     Vector2(-2,0),Vector2(-1,0),Vector2(0,0),Vector2(1,0)};
    verifyPositions<Scalar>(position,realPositions," case with center at (0,0) and limited min/max_column");

    //case 6 no points due to colum/index restrictions
    position.clear();
    iterateCircle<Scalar>(Vector2(Scalar(0),Scalar(0)),2,Eigen::Index(4),Eigen::Index(8),Eigen::Index(-8),Eigen::Index(8) ,[ &position ]( Eigen::Index x, Eigen::Index y )
    {
        position.push_back(Vector2(x,y));
    } );
    iterateCircle<Scalar>(Vector2(Scalar(0),Scalar(0)),2,Eigen::Index(-8),Eigen::Index(8),Eigen::Index(4),Eigen::Index(8) ,[ &position ]( Eigen::Index x, Eigen::Index y )
    {
        position.push_back(Vector2(x,y));
    } );
    realPositions = {};
    verifyPositions<Scalar>(position,realPositions," case with center at (0,0) and all points should be invalid due to column/row restrictions");

    // case 7, using different function overload
    position.clear();
    iterateCircle<Scalar>(Vector2(Scalar(0),Scalar(0)),2,Eigen::Index(4),Eigen::Index(4) ,[ &position ]( Eigen::Index x, Eigen::Index y )
    {
        position.push_back(Vector2(x,y));
    } );
    realPositions = {Vector2(0,0),Vector2(1,0),Vector2(0,1)};
    verifyPositions<Scalar>(position,realPositions," case with center at (0,0) usage of function overload setting min_ro/column to zeros");

}

TYPED_TEST( IteratorTest, polygonTest ) {
    using Scalar = TypeParam;
    using Vector2 = Vector2<Scalar>;
    std::vector<Vector2> position;
    Polygon<Scalar> polygon = createPolygon<Scalar>(0);
    //normal case in area x: -4 bis 4 and y: -4 bis 4, center (0,0) and radius 2,
    Eigen::Index row_min = -6;
    Eigen::Index row_max = 6;
    Eigen::Index col_min = -6;
    Eigen::Index col_max = 6;
    bool correctlyWorking = true;
    iteratePolygon<Scalar>(polygon,row_min,row_max,col_min,col_max ,[ &position ]( Eigen::Index x, Eigen::Index y )
    {
        position.push_back(Vector2(x,y));
    } );
    std::vector<Vector2> realPositions{{0,4},{1,4},{2,4},{3,4},
                                       {-3,3},{-1,3},{0,3},{1,3},{2,3},{3,3},
                                       {-4,2},{-3,2},{-2,2},{-1,2},{0,2},{1,2},{2,2},{3,2},
                                       {-4,1},{-3,1},{-2,1},{-1,1},
                                       {-4,0},{-3,0},{-2,0},{-1,0},{0,0},{1,0},
                                       {-4,-1},{-3,-1},{-2,-1},{-1,-1},
                                       {-2,-2}};//,{-1,-2}};
    correctlyWorking = verifyPositions<Scalar>(position,realPositions," case with 'random shape");
    if (!correctlyWorking) writeReportToFile(position, realPositions, polygon,row_min,row_max,col_min,col_max,"TestCasePolygonRandom.txt");
    // case Z
    polygon = createPolygon<Scalar>(1);
    position.clear();
    iteratePolygon<Scalar>(polygon,row_min,row_max,col_min,col_max,[ &position ]( Eigen::Index x, Eigen::Index y )
    {
        position.push_back(Vector2(x,y));
    } );
    realPositions={{-5,3},{-4,3},{-3,3},{-2,3},{-1,3},{0,3},
                  {0,2},
                  {-1,1},
                  {-2,0},
                  {-2,-1},
                  {-3,-2},{-2,-2},
                   {-4,-3},
                   {-5,-4},{-4,-4},{-3,-4},{-2,-4},{-1,-4},{0,-4}};
    correctlyWorking = verifyPositions<Scalar>(position,realPositions," case with Z-shape");
    if (!correctlyWorking) writeReportToFile(position, realPositions, polygon,row_min,row_max,col_min,col_max,"TestCasePolygonZShape.txt");

    // circle approximation
    polygon = createPolygon<Scalar>(2);
    position.clear();
    iteratePolygon<Scalar>(polygon,row_min,row_max,col_min,col_max,[ &position ]( Eigen::Index x, Eigen::Index y )
    {
        position.push_back(Vector2(x,y));
    } );
    realPositions={{-1.0,-5.0},{0.0,-5.0},
                   {-4.0,-4.0},{-3.0,-4.0},{-2.0,-4.0},{-1.0,-4.0},{0.0,-4.0},{1.0,-4.0},{2.0,-4.0},{3.0,-4.0},
                   {-4.0,-3.0},{-3.0,-3.0},{-2.0,-3.0},{-1.0,-3.0},{0.0,-3.0},{1.0,-3.0},{2.0,-3.0},{3.0,-3.0},
                   {-4.0,-2.0},{-3.0,-2.0},{-2.0,-2.0},{-1.0,-2.0},{0.0,-2.0},{1.0,-2.0},{2.0,-2.0},{3.0,-2.0},
                   {-5.0,-1.0},{-4.0,-1.0},{-3.0,-1.0},{-2.0,-1.0},{-1.0,-1.0},{0.0,-1.0},{1.0,-1.0},{2.0,-1.0},{3.0,-1.0},{4.0,-1.0},
                   {-5.0,0.0},{-4.0,0.0},{-3.0,0.0},{-2.0,0.0},{-1.0,0.0},{0.0,0.0},{1.0,0.0},{2.0,0.0},{3.0,0.0},{4.0,0.0},
                   {-4.0,1.0},{-3.0,1.0},{-2.0,1.0},{-1.0,1.0},{0.0,1.0},{1.0,1.0},{2.0,1.0},{3.0,1.0},
                   {-4.0,2.0},{-3.0,2.0},{-2.0,2.0},{-1.0,2.0},{0.0,2.0},{1.0,2.0},{2.0,2.0},{3.0,2.0},
                   {-4.0,3.0},{-3.0,3.0},{-2.0,3.0},{-1.0,3.0},{0.0,3.0},{1.0,3.0},{2.0,3.0},{3.0,3.0},
                   {-1.0,4.0},{0.0,4.0}};
    correctlyWorking = verifyPositions<Scalar>(position,realPositions," case with circle shape");
    if (!correctlyWorking) writeReportToFile(position, realPositions, polygon,row_min,row_max,col_min,col_max,"TestCaseCircleShape.txt");
    // u - shape
    /*polygon = createPolygon<Scalar>(3);
    position.clear();
    std::cout<<"u1"<<std::endl;
    iteratePolygon<Scalar>(polygon,row_min,row_max,col_min,col_max ,[ &position ]( Eigen::Index x, Eigen::Index y )
    {
        position.push_back(Vector2(x,y));
    } );
    std::cout<<"u2"<<std::endl;
    realPositions={{-3.0,-2.0},{-2.0,-2.0},{-1.0,-2.0},{0.0,-2.0},{1.0,-2.0},{4.0,-2.0},
                   {-3.0,-1.0},{1.0,-1.0},{4.0,-1.0},
                   {-3.0,0.0},{1.0,0.0},{4.0,0.0},
                   {-3.0,1.0},{1.0,1.0},{3.0,1.0},{4.0,1.0},
                   {-3.0,2.0},{1.0,2.0},{3.0,2.0},
                   {-3.0,3.0},{1.0,3.0},{2.0,3.0},{3.0,3.0}};
    correctlyWorking = verifyPositions<Scalar>(position,realPositions," case with u-shape");
    if (!correctlyWorking) writeReportToFile(position, realPositions, polygon,row_min,row_max,col_min,col_max,"TestCaseUShape.txt");*/

    // circle approximation with limited indexes
    polygon = createPolygon<Scalar>(2);
    position.clear();
    row_min =-4;
    row_max = 2;
    col_min = -3;
    col_max = 1;
    iteratePolygon<Scalar>(polygon,row_min,row_max,col_min,col_max ,[ &position ]( Eigen::Index x, Eigen::Index y )
    {
        position.push_back(Vector2(x,y));
    } );
    realPositions={{-4.0,-3.0},{-3.0,-3.0},{-2.0,-3.0},{-1.0,-3.0},{0.0,-3.0},{1.0,-3.0},
                    {-4.0,-2.0},{-3.0,-2.0},{-2.0,-2.0},{-1.0,-2.0},{0.0,-2.0},{1.0,-2.0},
                    {-4.0,-1.0},{-3.0,-1.0},{-2.0,-1.0},{-1.0,-1.0},{0.0,-1.0},{1.0,-1.0},
                    {-4.0,0.0},{-3.0,0.0},{-2.0,0.0},{-1.0,0.0},{0.0,0.0},{1.0,0.0}};

    correctlyWorking = verifyPositions<Scalar>(position,realPositions," case with circle shape and limited indexes");
    if (!correctlyWorking) writeReportToFile(position, realPositions, polygon,row_min,row_max,col_min,col_max,"TestCaseCircleShapeLimitedIndexes.txt");
    // u - shape limited index
    polygon = createPolygon<Scalar>(3);
    position.clear();
    iteratePolygon<Scalar>(polygon,row_min,row_max,col_min,col_max ,[ &position ]( Eigen::Index x, Eigen::Index y )
    {
        position.push_back(Vector2(x,y));
    } );
    realPositions={{-3.0,-2.0},{-2.0,-2.0},{-1.0,-2.0},{0.0,-2.0},{1.0,-2.0},
                   {-3.0,-1.0},{1.0,-1.0},
                   {-3.0,0.0},{1.0,0.0}};
    correctlyWorking = verifyPositions<Scalar>(position,realPositions," case with u-shape and limited indexes");
    if (!correctlyWorking) writeReportToFile(position, realPositions, polygon,row_min,row_max,col_min,col_max,"TestCaseUShapeLimitedIndexes.txt");

    // TODO: first shape and Z-shape fails, in circle index limitations lowest row seems to be missing
}

int main( int argc, char **argv )
{
    ros::init( argc, argv, "test_hector_iterators" );
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
