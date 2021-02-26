using RobotDynamics.MathUtilities;
using RobotDynamics.Robots;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JointHandler : MonoBehaviour
{
    public virtual void SetJointValue(HomogenousTransformation transformation, HomogenousTransformation nextTransformation, Link link, double q)
    {

    }
}
