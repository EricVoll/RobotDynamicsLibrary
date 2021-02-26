using RobotDynamics.MathUtilities;
using RobotDynamics.Robots;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RevoluteJointHandler : JointHandler
{
    public void Awake()
    {
        if(RootObject == null)
        {
            RootObject = this.gameObject;
        }
    }

    public override void SetJointValue(HomogenousTransformation transformation, HomogenousTransformation nextTransformation, Link link, double q)
    {
        Vector p = nextTransformation.GetPosition();
        RootObject.transform.localPosition = p.ToVector3();
        RotationMatrix R = nextTransformation.GetRotation();
        RootObject.transform.localRotation = R.ToUnityMatrix().rotation;
    }

    public GameObject RootObject;
}
