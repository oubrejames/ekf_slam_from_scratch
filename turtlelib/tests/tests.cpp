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
    turtlelib::Transform2D tf = {{0,1},turtlelib::deg2rad(90)};
    turtlelib::Transform2D ground_truth = {{-1,0},-turtlelib::deg2rad(90)};
    turtlelib::Transform2D inv_tf = tf.inv();

    turtlelib::Vector2D inv_tran = inv_tf.translation();
    double inv_rot = inv_tf.rotation();

    turtlelib::Vector2D out_tran = ground_truth.translation();
    double out_rot = ground_truth.rotation();

    REQUIRE_THAT(out_tran.x,  Catch::Matchers::WithinAbs(inv_tran.x, 0.001));
    REQUIRE_THAT(out_tran.y,  Catch::Matchers::WithinAbs(inv_tran.y, 0.0001));
    REQUIRE_THAT(out_rot,  Catch::Matchers::WithinAbs(inv_rot, 0.001));
}

TEST_CASE( "transform times equals", "[transform]" ) // James Oubre
{
    turtlelib::Transform2D tf1 = {{4,5},turtlelib::deg2rad(45)};
    turtlelib::Transform2D tf2 = {{6,7},turtlelib::deg2rad(80)};
    turtlelib::Transform2D ground_truth = {{3.29289,14.1924},turtlelib::deg2rad(125)};
    tf1*=tf2;

    turtlelib::Vector2D tran = tf1.translation();
    double rot = turtlelib::rad2deg(tf1.rotation());

    turtlelib::Vector2D out_tran = ground_truth.translation();
    double out_rot = turtlelib::rad2deg(ground_truth.rotation());

    REQUIRE_THAT(out_tran.x,  Catch::Matchers::WithinAbs(tran.x, 0.001));
    REQUIRE_THAT(out_tran.y,  Catch::Matchers::WithinAbs(tran.y, 0.001));
    REQUIRE_THAT(out_rot,  Catch::Matchers::WithinAbs(rot, 0.001));
}

TEST_CASE( "transform multiplication", "[transform]" ) // James Oubre
{
    turtlelib::Transform2D tf1 = {{4,5},turtlelib::deg2rad(45)};
    turtlelib::Transform2D tf2 = {{6,7},turtlelib::deg2rad(80)};
    turtlelib::Transform2D ground_truth = {{3.29289,14.1924},turtlelib::deg2rad(125)};
    turtlelib::Transform2D tf_out = tf1*tf2;

    turtlelib::Vector2D tran = tf_out.translation();
    double rot = turtlelib::rad2deg(tf_out.rotation());

    turtlelib::Vector2D out_tran = ground_truth.translation();
    double out_rot = turtlelib::rad2deg(ground_truth.rotation());

    REQUIRE_THAT(out_tran.x,  Catch::Matchers::WithinAbs(tran.x, 0.001));
    REQUIRE_THAT(out_tran.y,  Catch::Matchers::WithinAbs(tran.y, 0.001));
    REQUIRE_THAT(out_rot,  Catch::Matchers::WithinAbs(rot, 0.001));
}

TEST_CASE( "Twist2D transformation", "[Twist2d]" ) // James Oubre
{
    turtlelib::Transform2D tf1 = {{4,5},turtlelib::deg2rad(45)};
    turtlelib::Twist2D V_in = {6,7,8};
    turtlelib::Twist2D ground_truth = {6.0 , 29.2929, -13.3934};
    turtlelib::Twist2D V_out = tf1(V_in);

    REQUIRE_THAT(V_out.x,  Catch::Matchers::WithinAbs(ground_truth.x, 0.001));
    REQUIRE_THAT(V_out.y,  Catch::Matchers::WithinAbs(ground_truth.y, 0.001));
    REQUIRE_THAT(V_out.w,  Catch::Matchers::WithinAbs(ground_truth.w, 0.001));
}

TEST_CASE( "Rotation", "[transform]" ) // James Oubre
{
   double ang = 25.0;
   turtlelib::Transform2D tf = turtlelib::Transform2D(ang);
   REQUIRE(tf.rotation() == ang);
}

TEST_CASE( "Operator () for Vector2D", "[transform]" ) { // Yin, Hang
   float my_x = 2;
   float my_y = 3;
   float my_ang = turtlelib::PI;
   turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
   turtlelib::Vector2D v = {2,2};
   turtlelib::Vector2D result = Ttest(v);

    REQUIRE_THAT(result.y,  Catch::Matchers::WithinAbs(1.0, 0.001));
    REQUIRE_THAT(result.x,  Catch::Matchers::WithinAbs(0.0, 0.001));
}

TEST_CASE( "Rotation and Translation", "[transform]" ) { // Hughes, Katie
   float my_x = 2;
   float my_y = 3;
   float my_ang = 180;
   turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
   REQUIRE( Ttest.rotation() == my_ang);
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
   REQUIRE_THAT(norm_ang0,  Catch::Matchers::WithinAbs(turtlelib::PI, 0.001));

   double ang1 = -turtlelib::PI;
   double norm_ang1 = turtlelib::normalize_angle(ang1);
   REQUIRE_THAT(norm_ang1,  Catch::Matchers::WithinAbs(turtlelib::PI, 0.001));

   double ang2 = 0.0;
   double norm_ang2 = turtlelib::normalize_angle(ang2);
   REQUIRE_THAT(norm_ang2,  Catch::Matchers::WithinAbs(0.0, 0.001));

   double ang3 = -turtlelib::PI/4;
   double norm_ang3 = turtlelib::normalize_angle(ang3);
   REQUIRE_THAT(norm_ang3,  Catch::Matchers::WithinAbs(-turtlelib::PI/4, 0.001));

   double ang4 = 3*turtlelib::PI/2;
   double norm_ang4 = turtlelib::normalize_angle(ang4);
   REQUIRE_THAT(norm_ang4,  Catch::Matchers::WithinAbs(turtlelib::PI/2, 0.001));
   
   double ang5 = -5*turtlelib::PI/2;
   double norm_ang5 = turtlelib::normalize_angle(ang5);
   REQUIRE_THAT(norm_ang5,  Catch::Matchers::WithinAbs(-turtlelib::PI/2, 0.001));
}

TEST_CASE("Vector2D *=", "[Vector2D]"){ //James Oubre
    turtlelib::Vector2D in = {2, 3};
    double rhs = 3.0;
    in*=rhs;
    turtlelib::Vector2D expected = {6, 9};
    REQUIRE_THAT(in.x,  Catch::Matchers::WithinAbs(expected.x, 0.001));
    REQUIRE_THAT(in.y,  Catch::Matchers::WithinAbs(expected.y, 0.001));
}

TEST_CASE("Vector2D *", "[Vector2D]"){ //James Oubre
    turtlelib::Vector2D in = {2, 3};
    double rhs = 3.0;
    turtlelib::Vector2D out1=in*rhs;
    turtlelib::Vector2D out2=rhs*in;
    turtlelib::Vector2D expected = {6, 9};
    REQUIRE_THAT(out1.x,  Catch::Matchers::WithinAbs(expected.x, 0.001));
    REQUIRE_THAT(out1.y,  Catch::Matchers::WithinAbs(expected.y, 0.001));
    REQUIRE_THAT(out2.x,  Catch::Matchers::WithinAbs(expected.x, 0.001));
    REQUIRE_THAT(out2.y,  Catch::Matchers::WithinAbs(expected.y, 0.001));
}

TEST_CASE("Vector2D +=", "[Vector2D]"){ //James Oubre
    turtlelib::Vector2D in = {2, 3};
    turtlelib::Vector2D rhs = {4, 5};
    in+=rhs;
    turtlelib::Vector2D expected = {6, 8};
    REQUIRE_THAT(in.x,  Catch::Matchers::WithinAbs(expected.x, 0.001));
    REQUIRE_THAT(in.y,  Catch::Matchers::WithinAbs(expected.y, 0.001));
}

TEST_CASE("Vector2D +", "[Vector2D]"){ //James Oubre
    turtlelib::Vector2D in = {2, 3};
    turtlelib::Vector2D rhs = {4, 5};
    turtlelib::Vector2D out=in+rhs;
    turtlelib::Vector2D expected = {6, 8};
    REQUIRE_THAT(out.x,  Catch::Matchers::WithinAbs(expected.x, 0.001));
    REQUIRE_THAT(out.y,  Catch::Matchers::WithinAbs(expected.y, 0.001));
}

TEST_CASE("Vector2D -=", "[Vector2D]"){ //James Oubre
    turtlelib::Vector2D in = {2, 3};
    turtlelib::Vector2D rhs = {4, 5};
    in-=rhs;
    turtlelib::Vector2D expected = {-2, -2};
    REQUIRE_THAT(in.x,  Catch::Matchers::WithinAbs(expected.x, 0.001));
    REQUIRE_THAT(in.y,  Catch::Matchers::WithinAbs(expected.y, 0.001));
}

TEST_CASE("Vector2D -", "[Vector2D]"){ //James Oubre
    turtlelib::Vector2D in = {2, 3};
    turtlelib::Vector2D rhs = {4, 5};
    turtlelib::Vector2D out=in-rhs;
    turtlelib::Vector2D expected = {-2, -2};
    REQUIRE_THAT(out.x,  Catch::Matchers::WithinAbs(expected.x, 0.001));
    REQUIRE_THAT(out.y,  Catch::Matchers::WithinAbs(expected.y, 0.001));
}

TEST_CASE("magnitude()", "[Vector2D]"){ //James Oubre
    turtlelib::Vector2D in = {2, 3};
    double expected = 3.60555;
    REQUIRE_THAT(in.magnitude(),  Catch::Matchers::WithinAbs(expected, 0.001));
}

TEST_CASE("angle()", "[double]"){ //James Oubre
    turtlelib::Vector2D lhs = {2, 3};
    turtlelib::Vector2D rhs = {4, 5};
    double expected = 0.086738;
    REQUIRE_THAT(angle(lhs,rhs),  Catch::Matchers::WithinAbs(expected, 0.001));
}

TEST_CASE("dot()", "[double]"){ //James Oubre
    turtlelib::Vector2D lhs = {2, 3};
    turtlelib::Vector2D rhs = {4, 5};
    double expected = 23;
    REQUIRE_THAT(dot(lhs, rhs),  Catch::Matchers::WithinAbs(expected, 0.001));
}

TEST_CASE("integrate_twist() - Rotation and Translation", "[Transform2D]"){ //James Oubre
    turtlelib::Twist2D in = {1, 2, 3};

    turtlelib::Transform2D expected = {{0.303849, 3.44381}, 1};
    turtlelib::Vector2D expected_trans = expected.translation();
    double expected_rot = expected.rotation();

    turtlelib::Transform2D out = turtlelib::integrate_twist(in);
    turtlelib::Vector2D out_trans = out.translation();
    double out_rot = out.rotation();

    CHECK_THAT(out_trans.x,  Catch::Matchers::WithinAbs(expected_trans.x, 0.001));
    CHECK_THAT(out_trans.y,  Catch::Matchers::WithinAbs(expected_trans.y, 0.001));
    CHECK_THAT(out_rot,  Catch::Matchers::WithinAbs(expected_rot, 0.001));
}

TEST_CASE("integrate_twist() - Pure Translation", "[Transform2D]"){ //James Oubre
    turtlelib::Twist2D in = {0, 2, 3};

    turtlelib::Transform2D expected = {{2, 3}, 0.0};
    turtlelib::Vector2D expected_trans = expected.translation();
    double expected_rot = expected.rotation();

    turtlelib::Transform2D out = turtlelib::integrate_twist(in);
    turtlelib::Vector2D out_trans = out.translation();
    double out_rot = out.rotation();

    CHECK_THAT(out_trans.x,  Catch::Matchers::WithinAbs(expected_trans.x, 0.001));
    CHECK_THAT(out_trans.y,  Catch::Matchers::WithinAbs(expected_trans.y, 0.001));
    CHECK_THAT(out_rot,  Catch::Matchers::WithinAbs(expected_rot, 0.001));
}

TEST_CASE("integrate_twist() - Pure Rotation", "[Transform2D]"){ //James Oubre
    turtlelib::Twist2D in = {1, 0, 0};

    turtlelib::Transform2D expected = {{0, 0}, 1};
    turtlelib::Vector2D expected_trans = expected.translation();
    double expected_rot = expected.rotation();

    turtlelib::Transform2D out = turtlelib::integrate_twist(in);
    turtlelib::Vector2D out_trans = out.translation();
    double out_rot = out.rotation();

    CHECK_THAT(out_trans.x,  Catch::Matchers::WithinAbs(expected_trans.x, 0.001));
    CHECK_THAT(out_trans.y,  Catch::Matchers::WithinAbs(expected_trans.y, 0.001));
    CHECK_THAT(out_rot,  Catch::Matchers::WithinAbs(expected_rot, 0.001));
}