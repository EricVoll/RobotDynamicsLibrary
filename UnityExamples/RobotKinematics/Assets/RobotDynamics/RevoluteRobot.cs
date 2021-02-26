using RobotDynamics.Controller;
using RobotDynamics.MathUtilities;
using RobotDynamics.Robots;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RevoluteRobot : RobotBase
{
    public List<double> angles;
    public GameObject Target;


    public bool enableController = true;

    public bool useLastQAsInitValue = true;

    public int MaxIter = 100;

    // Start is called before the first frame update
    void Start()
    {
        Robot = new FanucCR7();
        Robot.AttachJointController(Kp, 0.01f);
        Robot.JointController.jointsChangedEvent += ControlledJointsChanged;
    }

    private void ControlledJointsChanged(object sender, JointsChangedEventArgs e)
    {
        if (!enableController) return;

        if (e.DidConverge)
            SetQ(e.joint_values);
    }



    Vector3 lastPos;
    Quaternion lastRot;
    double[] last_q;

    public double[] q;

    // Update is called once per frame
    void Update()
    {
        //SetQ(q);

        if (Target.transform.localPosition != lastPos || Target.transform.localRotation != lastRot)
        {
            lastPos = Target.transform.localPosition;
            lastRot = Target.transform.localRotation;

            //Inverse Kinematics
            Vector r_des = Target.transform.localPosition.ToVector();
            RotationMatrix C_des = Target.transform.EulerAnglesToRotationMatrix();

            var result = Robot.ComputeInverseKinematics(r_des, C_des, Lambda, Alpha, MaxIter, 0.1f, useLastQAsInitValue ? last_q : null);

            if (result.DidConverge)
            {
                last_q = result.q;

                if (!enableController)
                {
                    SetQ(result.q);
                }
            }
        }

        Robot.JointController.ReportNewFrame(Time.deltaTime);
    }

    #region Conversions



    
    #endregion
}
