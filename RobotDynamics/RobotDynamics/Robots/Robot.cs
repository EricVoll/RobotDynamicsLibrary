using RobotDynamics.MathUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

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


        public Robot AddJoint(char axe, Vector offset)
        {
            Link link = new Link(axe, offset);
            Links.Add(link);
            return this;
        }


        public List<HomogenousTransformation> ComputerForwardKinematics(double[] q)
        {
            List<HomogenousTransformation> hs = new List<HomogenousTransformation>();
            hs.Add(T_I0);
            for (int i = 0; i < Links.Count; i++)
            {
                hs.Add(new HomogenousTransformation(hs.Last() * Links[i].GetTransformation(q[i])));
            }
            return hs;
        }

        public double[] ComputeInverseKindematics(Vector I_r_IE, RotationMatrix C_IE_des, double lambda = 0.001f, double alpha = 0.05f, int max_it = 100)
        {
            var q = new Matrix(new double[Links.Count, 1]).ToVectorArray();
            int it = 0;
            var dxe = new Matrix(new double[6, 1]);
            float tol = 0.1f;
            bool loosendUpOnce = false;

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
                    it /= 2;
                    tol *= 10;
                    loosendUpOnce = true;
                }

                it++;
            }

            return bestQ;
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

            var T_IE = ComputerForwardKinematics(q).Last();
            Vector r_I_IE = T_IE.GetPosition();
            I_R_E_current = T_IE.GetRotation();


            List<double[]> j = new List<double[]>();
            for (int i = 0; i < Links.Count; i++)
            {
                j.Add(Vector.Cross(R_Ik[i] * n_k[i], r_I_IE - r_Ik[i]).ToArray());
            }

            J_P = new Matrix(j.ToArray());



            j = new List<double[]>();
            for (int i = 0; i < Links.Count; i++)
            {
                j.Add((R_Ik[i] * n_k[i]).ToArray());
            }
            J_R = new Matrix(j.ToArray());

            I_r_IE_current = r_I_IE;
        }


        #endregion

    }




    public class Link
    {
        public Link(char axe, Vector offset)
        {
            this.axe = axe;
            this.offset = offset;
        }

        char axe;
        Vector offset;

        double lastQ = -100;
        HomogenousTransformation lastHT = null;
        public HomogenousTransformation GetTransformation(double q)
        {
            if (lastQ == q) return lastHT;
            var HT = new HomogenousTransformation(new RotationMatrix(q, axe), offset);
            lastHT = HT;
            return HT;
        }

        public Vector GetN()
        {
            if (axe == 'x') return new Vector(1, 0, 0);
            if (axe == 'y') return new Vector(0, 1, 0);
            if (axe == 'z') return new Vector(0, 0, 1);
            return null;
        }
    }


}