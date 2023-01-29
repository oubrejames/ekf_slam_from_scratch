#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/rigid2d.hpp"

TEST_CASE( "Numbers are almost equal", "[bool]" ) // James Oubre
{
    REQUIRE( turtlelib::almost_equal(1, 1.002));
}

TEST_CASE( "Deg to rads", "[double]" ) // James Oubre
{
    double deg = 33;
    double rad = 0.575959;
    double rad_test = turtlelib::deg2rad(deg);
    REQUIRE( turtlelib::almost_equal(rad_test, rad));
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

    REQUIRE_THAT(out_tran.x,  Catch::Matchers::WithinRel(inv_tran.x, 0.001));
    REQUIRE_THAT(out_tran.y,  Catch::Matchers::WithinRel(inv_tran.y, 0.001));
    REQUIRE_THAT(out_rot,  Catch::Matchers::WithinRel(inv_rot, 0.001));
}

TEST_CASE( "transform times equals", "[transform]" ) // James Oubre
{
    turtlelib::Transform2D tf1 = {{4,5},45};
    turtlelib::Transform2D tf2 = {{6,7},80};
    turtlelib::Transform2D ground_truth = {{3.29289,14.1924},125};
    tf1*=tf2;

    turtlelib::Vector2D tran = tf1.translation();
    double rot = turtlelib::rad2deg(tf1.rotation());

    turtlelib::Vector2D out_tran = ground_truth.translation();
    double out_rot = turtlelib::rad2deg(ground_truth.rotation());

    REQUIRE_THAT(out_tran.x,  Catch::Matchers::WithinRel(tran.x, 0.001));
    REQUIRE_THAT(out_tran.y,  Catch::Matchers::WithinRel(tran.y, 0.001));
    REQUIRE_THAT(out_rot,  Catch::Matchers::WithinRel(rot, 0.001));
}

TEST_CASE( "transform multiplication", "[transform]" ) // James Oubre
{
    turtlelib::Transform2D tf1 = {{4,5},45};
    turtlelib::Transform2D tf2 = {{6,7},80};
    turtlelib::Transform2D ground_truth = {{3.29289,14.1924},125};
    turtlelib::Transform2D tf_out = tf1*tf2;

    turtlelib::Vector2D tran = tf_out.translation();
    double rot = turtlelib::rad2deg(tf_out.rotation());

    turtlelib::Vector2D out_tran = ground_truth.translation();
    double out_rot = turtlelib::rad2deg(ground_truth.rotation());

    REQUIRE( turtlelib::almost_equal(tran.x, out_tran.x, 1.0e-6));
    REQUIRE( turtlelib::almost_equal(tran.y, out_tran.y, 1.0e-6));
    REQUIRE( turtlelib::almost_equal(rot, out_rot, 1.0e-6));
}

TEST_CASE( "Twist2D transformation", "[Twist2d]" ) // James Oubre
{
    turtlelib::Transform2D tf1 = {{4,5},45};
    turtlelib::Twist2D V_in = {6,7,8};
    turtlelib::Twist2D ground_truth = {6.0 , 29.2929, -13.3934};
    turtlelib::Twist2D V_out = tf1(V_in);

    REQUIRE( turtlelib::almost_equal(V_out.w, ground_truth.w, 1.0e-6));
    REQUIRE( turtlelib::almost_equal(V_out.x, ground_truth.x, 1.0e-6));
    REQUIRE( turtlelib::almost_equal(V_out.y, ground_truth.y, 1.0e-6));
}

TEST_CASE( "Rotation", "[transform]" ) // James Oubre
{
   double ang = 25.0;
   turtlelib::Transform2D tf = turtlelib::Transform2D(ang);
   REQUIRE(tf.rotation() == turtlelib::deg2rad(ang));

}
 

TEST_CASE( "Operator () for Vector2D", "[transform]" ) { // Yin, Hang
   float my_x = 2;
   float my_y = 3;
   float my_ang = 180;
   turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
   turtlelib::Vector2D v = {2,2};
   turtlelib::Vector2D result = Ttest(v);
   REQUIRE( turtlelib::almost_equal(result.x, 0.0, 1.0e-5) );
   REQUIRE( turtlelib::almost_equal(result.y, 1.0, 1.0e-5) );
}

TEST_CASE( "Rotation and Translation", "[transform]" ) { // Hughes, Katie
   float my_x = 2;
   float my_y = 3;
   float my_ang = 180;
   turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
   REQUIRE( Ttest.rotation() == turtlelib::deg2rad(my_ang));
   REQUIRE( Ttest.translation().x == my_x);
   REQUIRE( Ttest.translation().y == my_y);
}

TEST_CASE("Translation()","[transform]"){ // Marno, Nel
   double test_x = 4.20;
   double test_y = 6.9;
   turtlelib::Transform2D T_test{{test_x,test_y}};
   REQUIRE(T_test.translation().x == test_x);
   REQUIRE(T_test.translation().y == test_y);
}

TEST_CASE( "normalize_angle()", "[double]" ) // James Oubre
{
   double ang0 = turtlelib::PI;
   double norm_ang0 = turtlelib::normalize_angle(ang0);
   REQUIRE_THAT(norm_ang0,  Catch::Matchers::WithinRel(turtlelib::PI, 0.001));

   double ang1 = -turtlelib::PI;
   double norm_ang1 = turtlelib::normalize_angle(ang1);
   REQUIRE_THAT(norm_ang1,  Catch::Matchers::WithinRel(turtlelib::PI, 0.001));

   double ang2 = 0.0;
   double norm_ang2 = turtlelib::normalize_angle(ang2);
   REQUIRE_THAT(norm_ang2,  Catch::Matchers::WithinRel(0.0, 0.001));

   double ang3 = -turtlelib::PI/4;
   double norm_ang3 = turtlelib::normalize_angle(ang3);
   REQUIRE_THAT(norm_ang3,  Catch::Matchers::WithinRel(-turtlelib::PI/4, 0.001));

   double ang4 = 3*turtlelib::PI/2;
   double norm_ang4 = turtlelib::normalize_angle(ang4);
   REQUIRE_THAT(norm_ang4,  Catch::Matchers::WithinRel(turtlelib::PI/2, 0.001));
   
   double ang5 = -5*turtlelib::PI/2;
   double norm_ang5 = turtlelib::normalize_angle(ang5);
   REQUIRE_THAT(norm_ang5,  Catch::Matchers::WithinRel(-turtlelib::PI/2, 0.001));
}
