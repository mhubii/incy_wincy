@author Matthew Millard
@date 10 February 2019
@version 0

This is an illustrative example to show how to compute and apply contact and friction forces to a bouncing ball. In this example:

1. The state of the ball is used to evaluate the position and velocity of the point of contact between a sphere and a plane. This is a compliant model so the sphere is allowed to penetrate the plane. The point of contact is the point on the sphere that is deepest into the plane.
2. The kinematics of the contact point in the normal direction are used to compute contact forces using a Hunt-Crossley contact model.
3. The tangental velocity of the contact point is used to compute a coefficient of friction using a regularized friction model. This coefficient of friction is used to compute friction forces.
4. Normal and friction forces are put into the f_ext vector into the proper location
5. RBDL's ForwardDynamics function is used to calculate the state derivative
6. Numerical integration is used to compute a state trajectory. The sum of system energy less work is tracked: if everything has been done correctly this will remain mostly constant, only changing as a function of numerical error from the integrator.

# Incy Wincy
