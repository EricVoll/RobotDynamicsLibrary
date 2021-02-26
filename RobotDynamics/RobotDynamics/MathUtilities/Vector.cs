using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobotDynamics.MathUtilities
{
    public class Vector
    {
        public Vector() : this(0, 0, 0)
        {

        }
        public Vector(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }

        public void SetCoords(Vector point)
        {
            X = point.X;
            Y = point.Y;
            Z = point.Z;
        }

        public double Magnitude
        {
            get { return Math.Sqrt(X * X + Y * Y + Z * Z); }
        }

        public static Vector operator +(Vector a, Vector b)
        {
            return new Vector(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        }
        public static Vector operator -(Vector a, Vector b)
        {
            return new Vector(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        }
        public static Vector operator -(Vector a)
        {
            return new Vector(-a.X, -a.Y, -a.Z);
        }
        public static Vector operator *(double a, Vector b)
        {
            return new Vector(a * b.X, a * b.Y, a * b.Z);
        }
        public static double operator *(Vector a, Vector b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }
        public static Vector operator /(Vector a, double b)
        {
            return new Vector(a.X / b, a.Y / b, a.Z / b);
        }

        public static Vector Cross(Vector a, Vector b)
        {
            Vector r = new Vector();
            r[0] = a[1] * b[2] - a[2] * b[1];
            r[1] = a[2] * b[0] - a[0] * b[2];
            r[2] = a[0] * b[1] - a[1] * b[0];
            return r;
        }

        public static Vector operator *(Vector b, double a)
        {
            return a * b;
        }

        public double[] ToArray()
        {
            return new double[] { X, Y, Z };
        }

        public Matrix ToMatrix()
        {
            return new Matrix(new double[,] { { X }, { Y }, { Z } });
        }

        public double this[int index]
        {

            // The get accessor.
            get
            {
                switch (index)
                {
                    case 0: return X;
                    case 1: return Y;
                    case 2: return Z;
                    default:
                        throw new IndexOutOfRangeException();
                }
            }

            // The set accessor.
            set
            {
                // set the value specified by index
                switch (index)
                {
                    case 0: X = value; break;
                    case 1: Y = value; break;
                    case 2: Z = value; break;
                    default:
                        throw new IndexOutOfRangeException();
                }
            }
        }
    }
}
