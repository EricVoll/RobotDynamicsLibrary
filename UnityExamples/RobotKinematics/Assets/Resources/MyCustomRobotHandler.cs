using RobotDynamics.MathUtilities;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MyCustomRobotHandler : RobotBase
{
    // Start is called before the first frame update
    void Awake()
    {
        Robot = new RobotDynamics.Robots.Robot();
        Robot
            .AddJoint('y', new Vector(0, 0.03293461, 0))
            .AddLinearJoint(new Vector(0, .5, 0), new Vector(0, 1, 0))
            .AddJoint('y', new Vector(-0.2681684, 0.01463607, -0.0003781915))
            .AddJoint('y', new Vector(0.1998537, -0.05284593, 0));
    }

    public GameObject Target;

    // Update is called once per frame
    void Update()
    {
        if (EnableForwardKinematics)
        {
            SetQ(ForwardKinematicsQ);
        }

        if (EnableInverseKinematics)
        {
            FollowTargetOneStep(Target);
        }

        Robot.JointController.ReportNewFrame(Time.deltaTime);
    }
}
