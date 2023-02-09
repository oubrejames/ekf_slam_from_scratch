# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input
- diff_drive - Class representing a differenttial drive object. Used to keep track the diff drive robot's location
and velocity

# Conceptual Questions
1. We need to be able to normalize Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the normalize functionality

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

   - Which of the methods would you implement and why?

| Design Implementation      | Pros | Cons     |
| :---        |    :----:   |          ---: |
| Define an external helper function to normalize the vector      | More decoupled code       | If you want to normalize more than a Vector2D you will have to impliment a much more complicated function that will distinguish between different input types   |
| Create a method within the Vector2D struct that returns a new normalized Vector2D   | Easy for user to get a new normalized vector        | Can only normalize Vector2D      |
| Create a method within the Vector2D struct that normalizes the current Vector2D   | Easy to normalize vector and can quickly modify existing vector        | Can only normalize Vector2D and will modify existing vector even if you don't want to      |

* I chose to to create a method within the Vector2D struct that returns a new normalized Vector2D.
The normalize function I created only can normalize a Vector2D so it makes sense to couple it with 
the struct instead of having it as a helper function. I also chose to return a new Vector2D instead 
of modify the existing one because the user may want to use the original vector after calculating
the normalized one.



2. What is the difference between a class and a struct in C++?
* A classes member variavles and functions are private by default while a struct's is public. Classes
may also inherit from other classes.



3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
* Vectord2D is a struct because all of its members are public (C.8) and it has no invariants (C.2)
* Transform2D is a class because it has multiple private members (C.8)



4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
* To avoid unintended conversions, you should declare some constructors explicit so that if not all 
arguments are passed, instead of converting them to the type of the first argument, it defaults the
missing value (C.46).



5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer
* Transform2D::inv() is declared const because no objects are changing their values. A new object is
instantiated and returned. For Transform2D::operator*=(), the value of the object itself is actually
changing so it would not work as a const. It is good to use const when possible because you cannot
have a race condition on a constant.

Worked With: Liz Metzger, Dilan Wijesinghe, Meg Sindelar, Marno Nel, Katie Hughes, Ava Zahedi, Nicolas Morales