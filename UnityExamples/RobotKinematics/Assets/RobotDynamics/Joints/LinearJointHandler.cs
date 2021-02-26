using RobotDynamics.MathUtilities;
using RobotDynamics.Robots;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LinearJointHandler : JointHandler
{
    public override void SetJointValue(HomogenousTransformation transformation, HomogenousTransformation nextTransformation, Link link, double q)
    {
        Vector p = transformation.GetPosition(); 
        RootObject.transform.localPosition = p.ToVector3();
        RotationMatrix R = transformation.GetRotation();
        RootObject.transform.localRotation = R.ToUnityMatrix().rotation;
        LinearJoint.transform.localPosition = (link.offset + link.linearMotionDirection * q).ToVector3();
    }

    /// <summary>
    /// Used for positioning the whole linear joint
    /// </summary>
    public GameObject RootObject;

    /// <summary>
    /// The linear joint is the thing that moves. Must be a child of the RootObject
    /// </summary>
    public GameObject LinearJoint;
}
