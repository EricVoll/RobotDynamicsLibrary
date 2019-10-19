using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobotDynamics.MathUtilities
{
    public class HomogenousTransformation : Matrix
    {
        public HomogenousTransformation() : base(new double[4, 4])
        {

        }

        public HomogenousTransformation(RotationMatrix R, Vector offset) : base(new double[4, 4])
        {
            double[,] m = new double[4, 4];
            for (int x = 0; x < 3; x++)
            {
                for (int y = 0; y < 3; y++)
                {
                    m[x, y] = R.matrix[x, y];
                }
            }
            for (int row = 0; row < 3; row++)
            {
                m[row, 3] = offset[row];
            }
            m[3, 3] = 1;

            matrix = m;
        }

        public HomogenousTransformation(Matrix m) : base(m.matrix)
        {
            if (m.NrCols != 4 || m.NrRows != 4)
            {
                throw new Exception("Matrix dimensions must be 4x4 for HT definitions");
            }
        }

        public Vector GetPosition()
        {
            return new Vector(matrix[0, 3], matrix[1, 3], matrix[2, 3]);
        }
        public RotationMatrix GetRotation()
        {
            double[,] m = new double[3, 3];
            for (int i = 0; i < 3; i++)
            {
                for (int q = 0; q < 3; q++)
                {
                    m[i, q] = matrix[i, q];
                }
            }
            return new RotationMatrix(m);
        }
    }
}
