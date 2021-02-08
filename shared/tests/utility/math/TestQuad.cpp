//Author: Shaan Arora

#include <Eigen/Core>
#include <ostream>
#include <vector>
#include <array>
#include <catch.hpp>
#include <Quad.h>

//Test suite/bed of points
static const std::array<std::vec<Eigen::Vector2f>, 10> p = {
    //empty vec, this is mainly for the domain error in getBoundingBox()
    std::vec<Eigen::Vector2f>{},
    //very small numbers
    std::vec<Eigen::Vector2f>{
        {0.5,0.5},
        {0.4,0.4},
        {0.3,0.3},
        {0.002, 0.002},
        {0.00001, 0.0001}
    },
    //very large numbers
    std::vec<Eigen::Vector2f>{
        {2342341,2342341},
        {111111.56,22222.56},
        {642023,234235},
        {234234.23423,232342.22342}
        {2232,5553}
    },
    //combination of small and larger
    std::vec<Eigen::Vector2f>{
        {0.000001, 111111},
        {1231423, 0.234},
        {0.12312, 533324},
        {234234.23423,0.00001}
        {0.3,642023}
    }
    //positive
    std::vec<Eigen::Vector2f>{
        {5.66,5},
        {6,6},
        {7.5,7.5},
        {10.0005,10.554},
        {11.11,11.12}
    }
    //negatives
    std::vec<Eigen::Vector2f>{
        {-2.767,-2},
        {-3,-3},
        {-4,-4.921},
        {-5.92,-5.034},
        {-6.5,-6.6}
    },
    //combination of positive & negative
    std::vec<Eigen::Vector2f>{
        {-2.5, 7.5},
        {7.65, -9},
        {9,-1.32},
        {-2.345, 5.676},
        {-5.46, 5.445}
    }
    //mix of all of the above except emtpy vec
    std::vec<Eigen::Vector2f>{
        {-.000234, 982342},
        {4673, -0.2353},
        {-0.55, -0.66},
        {-0.000001, 1000500},
        {1523, -0.1019125}
    }
}

//unit test for getBoundingBox()
TEST_CASE("Test the getBoundingBox function", ["utility"], ["math"], ["geometry"]){
    //Make a Quad object,
    //Vector2f will be turned into Matrix<float,2,1>
    Quad<Eigen::Vector2f> quad;
    for(auto point : p)
    {
        INFO("Testing with, \n" << point);
        quad = getBoundingBox(point)
        INFO("Bounding box generated", quad);
        //require statments here
    }
}

//unit test for aspectRatio()

//unit test for overlapsHorizontally()

//unit test for checkCornersValid()
