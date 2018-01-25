auto collide_circle_in_circle = [] (RigidBody &inner, RigidBody &outer)
{
    // https://gamedev.stackexchange.com/questions/29650/circle-inside-circle-collision
    // https://en.wikipedia.org/wiki/Minkowski_addition#Convex_hulls_of_Minkowski_sums
    
    // https://wildbunny.co.uk/blog/2011/04/20/collision-detection-for-dummies/
};

// the key here is that i need the minkowski DIFFERENCE
// because i want the circle to stay inside the outer circle
// collide_circle_in_circle(body, container);