using RobotDynamics.MathUtilities;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public interface IJoint
{
    void SetJointValue(Vector3 rootPosition, Quaternion rootRotation, Vector jointPos);
}
