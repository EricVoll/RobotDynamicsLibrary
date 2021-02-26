using RobotDynamics.Controller;
using RobotDynamics.MathUtilities;
using RobotDynamics.Robots;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LinearRobot : RobotBase
{
    public List<double> angles;
    public GameObject Target;

    public bool enableController = true;

    public bool useLastQAsInitValue = true;

    public int MaxIter = 100;

    // Start is called before the first frame update
    void Start()
    {
        Robot = new RobotDynamics.Robots.Robot();
        Robot.AddLinearJoint(new Vector(0, 0, 0), new Vector(1, 0, 0))
            .AddLinearJoint(new Vector(0, 0, 0), new Vector(0, 1, 0))
            .AddLinearJoint(new Vector(0, 0, 0), new Vector(0, 0, 1));
    }


    Vector3 lastPos;
    Quaternion lastRot;
    double[] last_q;

    public double[] q;

    // Update is called once per frame
    void Update()
    {
        if (q.Length != 3) return;
        SetQ(q);
    }

    #region Conversions


    public static Vector ToVector(Vector3 v)
    {
        return new Vector(v.x, v.y, v.z);
    }
    private RotationMatrix QuatToRotationMatrix(Quaternion quat)
    {
        double qw = quat.w;
        RotationMatrix matrix = null;
        var qn = new Matrix(new double[,] { { quat.x }, { quat.y }, { quat.z } });
        double[,] m = new double[,]
        {
            { 0, -quat.z, quat.y },
            { quat.z, 0 , -quat.x },
            { -quat.y, quat.x,0 }
        };
        matrix = new RotationMatrix(m);
        matrix = new RotationMatrix(((2 * qw * qw - 1) * Matrix.Eye(3) + 2 * qw * matrix + 2 * (qn * qn.Transpose())).matrix);
        return matrix;
    }

    /// <summary>
    /// Extracts the local Euler angles and returns a RotationMatrix
    /// </summary>
    /// <param name="transform"></param>
    /// <returns></returns>
    private RotationMatrix EulerAnglesToRotationMatrix(Transform transform)
    {

        double x = DegToRad(transform.localEulerAngles.x);
        double y = DegToRad(transform.localEulerAngles.y);
        double z = DegToRad(transform.localEulerAngles.z);


        Matrix R1 = new Matrix(new double[,]
       {
            { Math.Cos(y), 0, Math.Sin(y) },
            { 0,1,0 },
            {-Math.Sin(y), 0, Math.Cos(y) } });

        Matrix R2 = new Matrix(new double[,]
        {
            { 1,0,0 },
            {0, Math.Cos(x), -Math.Sin(x) },
            {0, Math.Sin(x), Math.Cos(x) } });
        Matrix R3 = new Matrix(new double[,]
        {
            { Math.Cos(z), -Math.Sin(z), 0 },
            {Math.Sin(z), Math.Cos(z), 0 },
            {0,0,1 } });

        RotationMatrix m = new RotationMatrix((R1 * R2 * R3).matrix);
        return m;
    }

    private double DegToRad(double deg)
    {
        return deg / 360.0 * 2 * Math.PI;
    }
    #endregion
}
