using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotBase : MonoBehaviour
{
    [Range(0, 1)]
    [Tooltip("A vlue used for pseudo inverses of matrices to stabalize in the case of singularities. Is added to the diagonal entries of the matrix before inverting.")]
    public float Lambda = 0.001f;
    [Range(0, 1)]
    [Tooltip("Step size factor in the numeric algorithm. [q = q + dq * alpha]")]
    public float Alpha = 0.05f;
    [Range(0, 5)]
    [Tooltip("Proportional controller parameter used for smoothing the angles")]
    public float Kp = 2;

    protected RobotDynamics.Robots.Robot Robot;
    public List<JointHandler> Joints;

    protected void SetQ(double[] q)
    {
        var transformations = Robot.ComputeForwardKinematics(q);

        for (int i = 0; i < transformations.Count - 1; i++)
        {
            Joints[i].SetJointValue(transformations[i], transformations[i + 1], Robot.Links[i], q[i]);
        }
    }
}
