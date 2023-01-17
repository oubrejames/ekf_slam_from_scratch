#include <catch2/catch_test_macros.hpp>
#include "turtlelib/rigid2d.hpp"

TEST_CASE( "Numbers are almost equal", "[almost_equal]" ) // James Oubre
{
    REQUIRE( turtlelib::almost_equal(1, 1.002));
}

TEST_CASE( "inverse", "[transform]" ) // James Oubre
{
    turtlelib::Transform2D tf = {{0,1},90};
    turtlelib::Transform2D ground_truth = {{-1,0},-90};
    turtlelib::Transform2D inv_tf = tf.inv();

    turtlelib::Vector2D inv_tran = inv_tf.translation();
    double inv_rot = inv_tf.rotation();

    turtlelib::Vector2D out_tran = ground_truth.translation();
    double out_rot = ground_truth.rotation();

    REQUIRE( turtlelib::almost_equal(inv_tran.x, out_tran.x, 1.0e-6));
    REQUIRE( turtlelib::almost_equal(inv_tran.y, out_tran.y, 1.0e-6));
    REQUIRE( turtlelib::almost_equal(inv_rot, out_rot, 1.0e-6));
}

TEST_CASE( "transform times equals", "[transform]" ) // James Oubre
{
    turtlelib::Transform2D tf1 = {{4,5},45};
    turtlelib::Transform2D tf2 = {{6,7},80};
    turtlelib::Transform2D ground_truth = {{3.29289,14.1924},125};
    tf1*=tf2;

    turtlelib::Vector2D tran = tf1.translation();
    double rot = tf1.rotation();

    turtlelib::Vector2D out_tran = ground_truth.translation();
    double out_rot = ground_truth.rotation();

    REQUIRE( turtlelib::almost_equal(tran.x, out_tran.x, 1.0e-6));
    REQUIRE( turtlelib::almost_equal(tran.y, out_tran.y, 1.0e-6));
    REQUIRE( turtlelib::almost_equal(rot, out_rot, 1.0e-6));
    REQUIRE( rot == out_rot);

}