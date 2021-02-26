using RobotDynamics.MathUtilities;
using RobotDynamics.Robots;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LinearJointHandler : JointHandler
{
    public override void SetJointValue(HomogenousTransformation transformation, HomogenousTransformation nextTransformation, Link link, double q)
    {
        RootObject.transform.localPosition = transformation.GetPosition().ToVector3();
        RootObject.transform.localRotation = transformation.GetRotation().ToUnityMatrix().rotation;
        LinearJoint.transform.localPosition = (link.offset + link.linearMotionDirection * q).ToVector3() / RootObject.transform.localScale.x;
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
