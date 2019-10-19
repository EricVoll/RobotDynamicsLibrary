using Microsoft.VisualStudio.TestTools.UnitTesting;
using RobotDynamics.MathUtilities;


namespace RobotDynamicsTests.MathUtilitiesTests
{
    [TestClass]
    public class MatrixTests
    {
        [TestMethod]
        public void OperatorNotEqualtest()
        {
            Matrix matrix = new Matrix(new double[3, 3] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } });
            Matrix inverse = matrix.GetInvert();
            Assert.IsTrue(matrix.Equals(inverse));
        }
        [TestMethod]
        public void OperatorNotEqualtest2()
        {
            Matrix matrix = new Matrix(new double[3, 3] { { 1, 2, 3 }, { 2, 3, 4 }, { 3, 4, 10 } });
            Matrix inverse = matrix.GetInvert();
            Matrix result = new Matrix(new double[3, 3] { { -2.8, 1.6, 0.2 }, { 1.6, -0.2, -0.4 }, { 0.2, -0.4, 0.2 } });
            Assert.IsTrue(inverse.Equals(result));
        }

        [TestMethod]
        public void TestPseudoInverse()
        {
            Matrix m = new Matrix(new double[,]
            {
                {1,2,3 },
                {3,2,-1 },
                {1,1,1 },
                {5,4,23 }
            });
            Matrix res = new Matrix(new double[,]
            {
                { -0.4148,    0.3340 ,  -0.0251 ,   0.0698 },
                { 0.6193  , -0.0702 ,   0.1309 ,  -0.0894 },
                { -0.0166  , -0.0610 ,  -0.0163  ,  0.0436 }
            });
            Matrix actual = m.GetDampedPseudoInverse(0.5);

            Assert.IsTrue(actual.Equals(res));
        }

        [TestMethod]
        public void Transpose()
        {
            Matrix m = new Matrix(new double[,]
            {
                {1,2,3 },
                {3,2,-1 },
                {1,1,1 }
            });

            Matrix b = new Matrix(new double[,]
            {
                {1,3,1 },
                {2,2,1 },
                {3,-1,1 }
            });

            Assert.IsTrue(b.Equals(m.Transpose()));
        }

        [TestMethod]
        public void Multiply()
        {
            Matrix m = new Matrix(new double[,]
            {
                {1,2,3 },
                {3,2,-1 },
                {1,1,1 }
            });

            Matrix b = new Matrix(new double[,]
            {
                {1,3,-1 },
                {1231,1,88 },
                {3,-1,4 }
            });

            Matrix res = new Matrix(new double[,]
            {
                {2472,2,187 },
                {2462, 12,169 },
                {1235,3,91 }
            });
            Matrix actual = m * b;
            Assert.IsTrue(actual.Equals(res));
        }


        [TestMethod]
        public void MultiplyNonSameSize()
        {
            Matrix m = new Matrix(new double[,]
            {
                {1,2,3 },
                {3,2,-1 },
                {1,1,1 },
                {5,4,23 }
            });

            Matrix b = new Matrix(new double[,]
            {
                {1,3,-1,4 },
                {1231,1,88,88 },
                {3,-1,4,2 }
            });

            Matrix res = new Matrix(new double[,]
            {
                {2472,2,187,186 },
                {2462, 12,169,186 },
                {1235,3,91,94 },
                {4998,-4,439,418 }
            });
            Matrix actual = m * b;
            Assert.IsTrue(actual.Equals(res));
        }

        [TestMethod]
        public void Duplicate()
        {
            Matrix m = new Matrix(new double[,]
            {
                {1,2,3 },
                {2,3,4 }
            });
            var b = m.GetDuplicate();

            m.matrix[1, 1] = 5;
            Assert.IsFalse(m.Equals(b));
        }
    }
}
