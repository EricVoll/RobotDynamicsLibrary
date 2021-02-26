using RobotDynamics.Controller;
using RobotDynamics.MathUtilities;
using RobotDynamics.Robots;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LinearRobot : RobotBase
{
    public GameObject Target;


    // Start is called before the first frame update
    void Awake()
    {
        Robot = new RobotDynamics.Robots.Robot();
        Robot
            .AddLinearJoint(new Vector(0, 0, 0), new Vector(0, 0, 1))
            .AddJoint('z', new Vector(0, 0, 0))
            .AddJoint('z', new Vector(0, 4, -1))
            .AddJoint('z', new Vector(0, 4, -1))
            .AddJoint('z', new Vector(0, 4, -1));
    }

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
