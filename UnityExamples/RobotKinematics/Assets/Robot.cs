using RobotDynamics.MathUtilities;
using RobotDynamics.Robots;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Robot : MonoBehaviour
{
    public List<GameObject> Joints;
    public List<double> angles;
    public GameObject Target;
    FanucCR7 CRobot = new FanucCR7();

    [Range(0,1)]
    public float Lambda = 0.001f;
    [Range(0,1)]
    public float Alpha = 0.05f;
    public int MaxIter = 100;

    // Start is called before the first frame update
    void Start()
    {

    }

    public static Vector ToVector(Vector3 v)
    {
        return new Vector(v.x, v.y, v.z);
    }

    // Update is called once per frame
    void Update()
    {

        //Inverse Kinematics
        Vector r_des = ToVector(Target.transform.localPosition);
        RotationMatrix C_des = QuatToRotationMatrix(Target.transform.localRotation);
        var q = CRobot.ComputeInverseKindematics(r_des, C_des, Lambda, Alpha, MaxIter);

        //Update robot
        SetQ(q);
    }

    private void SetQ(double[] q)
    {
        var transformations = CRobot.ComputerForwardKinematics(q);

        for (int i = 0; i < transformations.Count - 1; i++)
        {
            Vector p = transformations[i + 1].GetPosition();
            Joints[i].transform.localPosition = new Vector3((float)p.X, (float)p.Y, (float)p.Z);
            RotationMatrix R = transformations[i + 1].GetRotation();

            Joints[i].transform.localRotation = FromRotationMatrx(R).rotation;

            angles[i] = (float)q[i];
        }
        
    }

    private Matrix4x4 FromRotationMatrx(RotationMatrix R)
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
        matrix = new RotationMatrix( ((2 * qw * qw - 1) * Matrix.Eye(3) + 2 * qw * matrix + 2 * (qn * qn.Transpose())).matrix );
        return matrix;
    }
}
