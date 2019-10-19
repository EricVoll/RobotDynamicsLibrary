# RobotDynamicsLibrary
This library contains methods to perform forward/inverse kinematics 

Currently it only supports Robots, that have rotationary joints only. The mathematics model would allow translationary joints easily, but it has not been implemented yet.

# How to use
## Step 1: Config the robot
` Robot Ro = new Robot()
                .AddJoint('z', new Vector(0, 0, 300))
                .AddJoint('x', new Vector(0, 50, 330))
                .AddJoint('x', new Vector(0, 0, 440))
                .AddJoint('y', new Vector(0, 100, 35))
                .AddJoint('x', new Vector(0, 320, 0))
                .AddJoint('y', new Vector(0, 80, 0));

 Links = Ro.Links;`
 
 The `Robot` class has a `List<Link>` property. When creating an instance of a `Robot` object, the `AddJoint(...)` method allows to add joints to the kinematic chain of the robot. This method takes a char that defines the axis that the next (!) joint rotates around, and an offset `Vector` that defines the eukledian distance to the next joint. The Homogenous Transformation is then calculated from the rotation axes by getting the corresponding RotationMatrix and the offset vector is inserted at its corresponding place. The offset vector is defined relative to the coordinate system of the link.
 
The number of Links is freely selectable. The forward kinematics will work with any number of links, the inverse kinematics is currently limited to a 6 DOF robot, but will be extended to allow Multi-Task control.

## Step 2: Forward Kinematics
The `Robot` object has a Method called `ComputeForwardKinematics(double[] q)` which takes a vector `q` as an input. The method returns all `HomogenousTransformations` of all Joints relative to the initial frame, which allows to set the position and rotation of all joints. For that the `HomogenousTransformation` class has two methods `GetPosition()` and `GetRotation()`, which respectivley return a `Vector` (3x1) and a `RotationMatrix` (3x3) object. 

## Step 3: Inverse Kinematics
The `Robot` object has a method called `ComputeInverseKinematics(Vector x, RotationMatrix R, double alpha, double lambda, int maxIt)`, which returns an array `double[]` with the q values.
Vector x: The desired task-space position vector.
RotationMatrix R: The desired orientation as a RotationMatrix object.
double alpha: A scaling factor in the numeric solution to make it more stable in steps with big errors between the desired and current position.
double lambda: The damping factor for the pseudo Inverse of the Jacobian Matrix.
int maxIt: The number of max iterations the numerical solution is allowed to take.

After the maxIt was reached once, alpha is halved (making it even slower to converge but more precise and less error prone) and the algorithm is given maxIt / 2 more steps to find the optimal solution. 

# Example project:
The exmaple project uses Fanuc's CR7 collaborative robot. It is programmed in a way, that the user can move the task-space target position and orientation around and the robot will perform inverse kinematics to match its end-effector with the target. Factors such as alpha, lambda and maxIt can be set in the editor to see their effects.
![Example Screenshot](https://github.com/luchspeter/RobotDynamicsLibrary/blob/master/ReadmeRessources/UnityExampleScreenshot.png)
