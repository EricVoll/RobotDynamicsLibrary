using RobotDynamics.MathUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using RobotDynamics.Controller;

namespace RobotDynamics.Robots
{
    public class Robot
    {
        public Robot()
        {
            T_I0 = new HomogenousTransformation(Matrix.Eye(4));
        }

        private HomogenousTransformation T_I0;
        public List<Link> Links = new List<Link>();
        public JointController JointController { get; private set; }

        public Robot AddJoint(char axe, Vector offset)
        {
            Link link = Link.Revolute(axe, offset);
            Links.Add(link);
            return this;
        }
        public Robot AddLinearJoint(Vector offset, Vector linearMovementDirection)
        {
            Links.Add(Link.Linear(linearMovementDirection, offset));
            return this;
        }

        public void AttachJointController(float kp, float tolerance)
        {
            JointController = new JointController(kp, tolerance, Links.Count, Links.Select(x => x.Type).ToArray());
        }

        /// <summary>
        /// Computes the forward kinematics of the defined robot
        /// </summary>
        /// <param name="q">The current joint values</param>
        /// <returns></returns>
        public List<HomogenousTransformation> ComputeForwardKinematics(double[] q)
        {
            List<HomogenousTransformation> hs = new List<HomogenousTransformation>();
            hs.Add(T_I0);
            for (int i = 0; i < Links.Count; i++)
            {
                hs.Add(new HomogenousTransformation(hs.Last() * Links[i].GetTransformation(q[i])));
            }
            return hs;
        }

        public IterationResult ComputeInverseKinematics(Vector I_r_IE, RotationMatrix C_IE_des, double lambda = 0.001f, double alpha = 0.05f, int max_it = 100, float tol = 0.1f, double[] q_0 = null)
        {
            return ComputeInverseKinematics(I_r_IE, C_IE_des, Matrix.Eye(Links.Count) * alpha, lambda, max_it, tol, q_0);
        }

        /// <summary>
        /// Computes the inverse kinematics of the currently defined robot.
        /// </summary>
        /// <param name="I_r_IE">The desired position vector of the End-Effector frame defined in the Base frame</param>
        /// <param name="C_IE_des">The desired rotation matrix of the end-effector defined in the base frame</param>
        /// <param name="lambda">A value added to the main diagonal of matrices when calculating the pseudo inverse to make them more stable in the case of singularities</param>
        /// <param name="alpha">Step size to move the gradient per iteration</param>
        /// <param name="max_it">The maximum number of iterations</param>
        /// <returns></returns>
        public IterationResult ComputeInverseKinematics(Vector I_r_IE, RotationMatrix C_IE_des, Matrix alpha, double lambda = 0.001f, int max_it = 100, float tol = 0.1f, double[] q_0 = null)
        {
            double[] q;
            if (q_0 == null)
                q = new Matrix(new double[Links.Count, 1]).ToVectorArray();
            else
            {
                if (q_0.Length != Links.Count)
                {
                    throw new Exception("Invalid initial q passed into inverse kinematics");
                }
                q = q_0;
            }
            int it = 0;
            var dxe = new Matrix(new double[6, 1]);
            bool loosendUpOnce = false;
            IterationResult result = new IterationResult();

            double[] bestQ = q;
            double bestNorm = Double.MaxValue;

            while ((it == 0 || dxe.Norm() > tol) && it < max_it)
            {
                if (dxe.Norm() < bestNorm)
                {
                    bestQ = q;
                }

                GetJacobians(q, out Matrix J_P, out Matrix J_R, out Vector I_r_current, out RotationMatrix R_current);
                Matrix J = Matrix.Stack(J_P, J_R);
                Matrix J_pseudo = J.GetDampedPseudoInverse(lambda);

                Vector dr = I_r_IE - I_r_current;

                Vector dphi = new RotationMatrix((C_IE_des * R_current.Transpose()).matrix).ToAngleAxis();

                dxe = dr.ToMatrix().Stack(dphi.ToMatrix());

                var qd = (alpha * J_pseudo * dxe).ToVectorArray();

                for (int i = 0; i < q.Length; i++)
                {
                    q[i] += qd[i];
                }

                //If we reached our max_it we loosen up the exit-requirements to get a as good as possible result
                //But only do it once.
                if (it == max_it - 1 && !loosendUpOnce)
                {
                    result.numberOfIterationsPerfomred += max_it/2;
                    it /= 2;
                    tol *= 10;
                    loosendUpOnce = true;
                    result.DidLoosenUpTolerance = true;
                }
                if (it == max_it - 1 && loosendUpOnce)
                {
                    result.DidConverge = false;
                }

                it++;
            }

            result.q = bestQ;
            result.numberOfIterationsPerfomred += it;

            if (JointController != null)
            {
                JointController.ReportNewTargetJointValues(result.q, result.DidConverge);
            }

            return result;
        }


        #region GetJacobians

        /// <summary>
        /// Takes the current q vector as an inpute and returns the geometric Position Jacobian J_P, the geometric Rotation Jacobian J_R, the current task-space position r_IE and the current task-space end-effector rotatoin matrix
        /// <para>All are expressed in the I-Frame</para>
        /// </summary>
        /// <param name="q"></param>
        /// <param name="J_P"></param>
        /// <param name="J_R"></param>
        /// <param name="I_r_IE_current"></param>
        /// <param name="I_R_E_current"></param>
        private void GetJacobians(double[] q, out Matrix J_P, out Matrix J_R, out Vector I_r_IE_current, out RotationMatrix I_R_E_current)
        {
            List<HomogenousTransformation> T_k_k1 = new List<HomogenousTransformation>() { };
            for (int i = 0; i < Links.Count; i++)
            {
                T_k_k1.Add(Links[i].GetTransformation(q[i]));
            }

            List<HomogenousTransformation> T_I_k = new List<HomogenousTransformation>() { T_I0 };
            for (int i = 0; i < Links.Count; i++)
            {
                T_I_k.Add(new HomogenousTransformation(T_I_k[i] * T_k_k1[i]));
            }
            T_I_k.RemoveAt(0); //Remove the first transformation which was only added for a nicer for loop.

            List<RotationMatrix> R_Ik = new List<RotationMatrix>();
            List<Vector> r_Ik = new List<Vector>();
            foreach (var T in T_I_k)
            {
                R_Ik.Add(T.GetRotation());
                r_Ik.Add(T.GetPosition());
            }

            List<Vector> n_k = new List<Vector>();
            foreach (var link in Links)
            {
                n_k.Add(link.GetN());
            }

            var T_IE = ComputeForwardKinematics(q).Last();
            Vector r_I_IE = T_IE.GetPosition();
            I_R_E_current = T_IE.GetRotation();


            List<double[]> j = new List<double[]>();
            for (int i = 0; i < Links.Count; i++)
            {
                if (Links[i].Type == Link.JointType.Revolute)
                {
                    j.Add(Vector.Cross(R_Ik[i] * n_k[i], r_I_IE - r_Ik[i]).ToArray());
                }
                else if (Links[i].Type == Link.JointType.Linear)
                {
                    j.Add((R_Ik[i] * n_k[i]).ToArray());
                }
            }

            J_P = new Matrix(j.ToArray());



            j = new List<double[]>();
            for (int i = 0; i < Links.Count; i++)
            {

                if (Links[i].Type == Link.JointType.Revolute)
                {
                    j.Add((R_Ik[i] * n_k[i]).ToArray());
                }
                else if (Links[i].Type == Link.JointType.Linear)
                {
                    j.Add(new double[] { 0, 0, 0 });
                }
            }
            J_R = new Matrix(j.ToArray());

            I_r_IE_current = r_I_IE;
        }


        #endregion

    }

    public class IterationResult
    {
        public double[] q { get; set; }
        public bool DidLoosenUpTolerance { get; set; } = false;
        public bool DidConverge { get; set; } = true;
        public int numberOfIterationsPerfomred { get; set; }
    }

}