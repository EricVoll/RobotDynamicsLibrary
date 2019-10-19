using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobotDynamics.MathUtilities
{
    public class Matrix
    {
        public double[,] matrix { get; set; }
        public int NrRows { get { return matrix.GetLength(0); } }
        public int NrCols { get { return matrix.GetLength(1); } }

        public Matrix(double[,] matrix)
        {
            this.matrix = matrix;
        }

        /// <summary>
        /// Init by array of columns
        /// </summary>
        /// <param name="columns"></param>
        public Matrix(double[][] columns)
        {
            matrix = new double[columns[0].Length, columns.Length];
            for (int i = 0; i < columns.Length; i++)
            {
                for (int row = 0; row < columns[i].Length; row++)
                {
                    matrix[row, i] = columns[i][row];
                }
            }
        }

        #region Operators

        public static Vector operator *(Matrix a, Vector b)
        {
            double[] value = new double[3];
            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    value[r] += a.matrix[r, c] * b[c];
                }
            }
            return new Vector(value[0], value[1], value[2]);
        }

        public static Matrix operator *(Matrix a, Matrix b)
        {
            double[,] values = new double[a.NrRows, b.NrCols];
            for (int row = 0; row < a.NrRows; row++)
            {
                for (int col = 0; col < b.NrCols; col++)
                {
                    values[row, col] = 0;
                    for (int i = 0; i < a.NrCols; i++)
                    {
                        values[row, col] += a.matrix[row, i] * b.matrix[i, col];
                    }
                }
            }
            return new Matrix(values);
        }
        public static Matrix operator *(Matrix a, double factor)
        {
            Matrix b = a.GetDuplicate();
            for (int i = 0; i < a.NrRows; i++)
            {
                for (int q = 0; q < a.NrCols; q++)
                {
                    b.matrix[i, q] *= factor;
                }
            }
            return b;
        }
        public static Matrix operator *(double f, Matrix a)
        {
            return a * f;
        }
        public static Matrix operator +(Matrix a, Matrix b)
        {
            if (a.NrCols != b.NrCols || a.NrRows != b.NrRows)
            {
                throw new Exception("Dimensions of matrix do not match");
            }
            double[,] m = new double[a.NrRows, a.NrCols];
            for (int i = 0; i < a.NrRows; i++)
            {
                for (int q = 0; q < a.NrCols; q++)
                {
                    m[i, q] = a.matrix[i, q] + b.matrix[i, q];
                }
            }
            return new Matrix(m);
        }
        #endregion

        #region Members

        /// <summary>
        /// Creates a duplicate matrix that is not a reference copy
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        public Matrix GetDuplicate()
        {
            //We need to copy every entry because creating an instance with the constructor and array creates a reference to the array.
            //deep copy that stuff.
            double[,] m = new double[this.NrRows, this.NrCols];
            for (int i = 0; i < this.NrRows; i++)
            {
                for (int q = 0; q < this.NrCols; q++)
                {
                    m[i, q] = this.matrix[i, q];
                }
            }
            return new Matrix(m);
        }

        /// <summary>
        /// Inverts the Matrix
        /// </summary>
        /// <returns></returns>
        public Matrix GetInvert()
        {
            return MatrixInversion.InverseMatrix(this);
        }

        /// <summary>
        /// Transposes the Matrix
        /// </summary>
        /// <returns></returns>
        public Matrix Transpose()
        {
            double[,] t = new double[NrCols, NrRows];

            for (int row = 0; row < NrRows; row++)
            {
                for (int col = 0; col < NrCols; col++)
                {
                    t[col, row] = matrix[row, col];
                }
            }
            return new Matrix(t);
        }

        /// <summary>
        /// The damping makes the pseudo inverse more stable in close to singular cases
        /// Calling it with dambda = 0 results in a normal pseudo invers
        /// </summary>
        /// <param name="lambda"></param>
        /// <returns></returns>
        public Matrix GetDampedPseudoInverse(double lambda)

        {
            Matrix result = null;
            Matrix transpose = Transpose();
            if (NrRows >= NrCols)
            {
                result = (transpose * this + lambda * lambda * Matrix.Eye(NrCols)).GetInvert() * transpose;
            }
            if (NrCols > NrRows)
            {
                result = transpose * (this * transpose + lambda * lambda * Matrix.Eye(NrRows)).GetInvert();
            }
            return result;
        }

        /// <summary>
        /// OVerriden Equals operator which checks each entry of the matrix with a certain tolerance for numerical errors
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            Matrix b = (Matrix)obj;
            double tol = 0.0001;

            if (NrRows != b.NrRows || NrCols != b.NrCols) return false;

            for (int i = 0; i < NrRows; i++)
            {
                for (int q = 0; q < NrCols; q++)
                {
                    double v = b.matrix[i, q];
                    if (!(matrix[i, q] > v - tol && matrix[i, q] < v + tol)) return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Returns the L2 Norm of the array
        /// </summary>
        /// <returns></returns>
        public double Norm()
        {
            double nr = 0;
            for (int i = 0; i < NrRows; i++)
            {
                for (int q = 0; q < NrCols; q++)
                {
                    nr += Math.Pow(matrix[i, q], 2);
                }
            }
            return Math.Sqrt(nr);
        }

        /// <summary>
        /// Returns a one dimensional array with the first column if it only has one column
        /// </summary>
        /// <returns></returns>
        public double[] ToVectorArray()
        {
            if (NrCols == 1)
            {
                double[] v = new double[NrRows];
                for (int i = 0; i < NrRows; i++)
                {
                    v[i] = matrix[i, 0];
                }
                return v;
            }
            else
            {
                throw new Exception($"{nameof(ToVectorArray)} can only process nx1 matrices");
            }
        }

        /// <summary>
        /// Stacks the Matrix m underneath the current matrix.
        /// The result is only returned, not written to the current matrix
        /// </summary>
        /// <param name="m"></param>
        /// <returns></returns>
        public Matrix Stack(Matrix m)
        {
            return Matrix.Stack(this, m);
        }

        #endregion

        #region Static Methods

        /// <summary>
        /// Returns a EYE nxn Matrix of n - dimensions.
        /// </summary>
        /// <param name="n"></param>
        /// <returns></returns>
        public static Matrix Eye(int n)
        {
            double[,] m = new double[n, n];
            for (int i = 0; i < n; i++)
            {
                m[i, i] = 1;
            }

            return new Matrix(m);
        }


        public static Matrix Stack(Matrix Top, Matrix Bottom)
        {
            if (Top.NrCols != Bottom.NrCols)
            {
                throw new Exception("To stack matrices they have to have the same nr of cols");
            }
            double[,] m = new double[Top.NrRows + Bottom.NrRows, Top.NrCols];

            for (int row = 0; row < Top.NrRows; row++)
            {
                for (int col = 0; col < Top.NrCols; col++)
                {
                    m[row, col] = Top.matrix[row, col];
                }
            }

            for (int row = 0; row < Bottom.NrRows; row++)
            {
                for (int col = 0; col < Bottom.NrCols; col++)
                {
                    m[row + Top.NrRows, col] = Bottom.matrix[row, col];
                }
            }

            return new Matrix(m);
        }

        #endregion

    }
}
