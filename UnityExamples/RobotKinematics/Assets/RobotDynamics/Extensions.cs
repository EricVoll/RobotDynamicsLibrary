using RobotDynamics.MathUtilities;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Extensions {
    public static Vector3 ToVector3(this Vector vector)
    {
        return new Vector3((float)vector.X, (float)vector.Y, (float)vector.Z);
    }

    public static Vector ToVector(this Vector3 v)
    {
        return new Vector(v.x, v.y, v.z);
    }

    public static Matrix4x4 ToUnityMatrix(this RotationMatrix R)
    {
        Vector4 c0 = new Vector4();
        Vector4 c1 = new Vector4();
        Vector4 c2 = new Vector4();
        Vector4 c3 = new Vector4();
        for (int i = 0; i < 3; i++)
        {
            c0[i] = (float)R.matrix[i, 0];
            c1[i] = (float)R.matrix[i, 1];
            c2[i] = (float)R.matrix[i, 2];
        }
        c3[3] = 1;
        var m = new Matrix4x4(c0, c1, c2, c3);
        return m;
    }

    /// <summary>
    /// Extracts the local Euler angles and returns a RotationMatrix
    /// </summary>
    /// <param name="transform"></param>
    /// <returns></returns>
    public static RotationMatrix EulerAnglesToRotationMatrix(this Transform transform)
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

    public static double DegToRad(double deg)
    {
        return deg / 360.0 * 2 * Math.PI;
    }
}
