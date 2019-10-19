using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobotDynamics.MathUtilities
{
    public class RotationMatrix : Matrix
    {
        public RotationMatrix() : base(new double[3, 3])
        {

        }

        public RotationMatrix(double q, char axe) : base(new double[3, 3])
        {
            double[,] m = new double[3, 3];
            if (axe == 'x')
            {
                m = new double[3, 3]
                {
                    { 1,0,0 },
                    { 0, Math.Cos(q), -Math.Sin(q)},
                    { 0, Math.Sin(q),  Math.Cos(q)}
                };
            }
            if (axe == 'y')
            {
                m = new double[,]
                {
                    {Math.Cos(q),0, Math.Sin(q) },
                    { 0,1,0 },
                    { -Math.Sin(q), 0, Math.Cos(q) }
                };
            }
            if (axe == 'z')
            {
                m = new double[,]
                {
                    { Math.Cos(q), -Math.Sin(q), 0 },
                    { Math.Sin(q), Math.Cos(q), 0 },
                    { 0,0,1}
                };
            }

            matrix = m;
        }

        public RotationMatrix(double[,] matrix) : base(matrix)
        {
            //Just please dont add matrices with dimensions not equal to 3x3
        }

        public double[] ToQuaternion()
        {
            double v0 = 0.5 * Math.Sqrt(1 + matrix[0, 0] + matrix[1, 1] + matrix[2, 2]);
            double v1 = Math.Sign(matrix[2, 1] - matrix[1, 2]) * Math.Sqrt(matrix[0, 0] - matrix[1, 1] - matrix[2, 2] + 1);
            double v2 = Math.Sign(matrix[0, 2] - matrix[2, 0]) * Math.Sqrt(matrix[1, 1] - matrix[2, 2] - matrix[0, 0] + 1);
            double v3 = Math.Sign(matrix[1, 0] - matrix[0, 1]) * Math.Sqrt(matrix[2, 2] - matrix[0, 0] - matrix[1, 1] + 1);
            return new double[] { v0, v1, v2, v3 };
        }

        /// <summary>
        /// Converts the matrix to AngleAxisRepresentation
        /// </summary>
        /// <returns></returns>
        public Vector ToAngleAxis()
        {

            double angle = Math.Acos((matrix[0, 0] + matrix[1, 1] + matrix[2, 2] - 1) / 2);

            if (Math.Abs(angle) < 0.01)
            {
                return new Vector(0, 0, 0);
            }
            else
            {
                Vector phi = new Vector(matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]) * (1 / (2 * Math.Sin(angle)));
                phi = phi * angle;
                return phi;
            }
        }

        new public RotationMatrix Transpose()
        {
            return new RotationMatrix(((Matrix)this).Transpose().matrix);
        }
    }
}
