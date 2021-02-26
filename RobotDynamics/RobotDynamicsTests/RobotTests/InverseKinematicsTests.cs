using Microsoft.VisualStudio.TestTools.UnitTesting;
using RobotDynamics.MathUtilities;
using RobotDynamics.Robots;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotDynamicsTests.RobotTests
{
    [TestClass]
    public class InverseKinematicsTests
    {
        [TestMethod]
        public void LinearRobot()
        {

            var Robot = new RobotDynamics.Robots.Robot();
            Robot.AddLinearJoint(new Vector(0, 0, 0), new Vector(1, 0, 0))
                .AddLinearJoint(new Vector(0, 0, 0), new Vector(0, 1, 0))
                .AddLinearJoint(new Vector(0, 0, 0), new Vector(0, 0, 1));

            var result = Robot.ComputeInverseKinematics(new Vector(0, 0, 0), new RotationMatrix(Matrix.Eye(3).matrix));
            Assert.IsTrue(result.DidConverge);
            Assert.IsTrue(result.q.All(x => x < 0.001f));
        }

        [TestMethod]
        public void LinearRobot_2()
        {

            var Robot = new RobotDynamics.Robots.Robot();
            Robot.AddLinearJoint(new Vector(0, 0, 0), new Vector(1, 0, 0))
                .AddLinearJoint(new Vector(0, 0, 0), new Vector(0, 1, 0))
                .AddLinearJoint(new Vector(0, 0, 0), new Vector(0, 0, 1));

            var result = Robot.ComputeInverseKinematics(new Vector(1, 0, 0), new RotationMatrix(Matrix.Eye(3).matrix), 0.001, 0.05, 100, 0.001f);
            Assert.IsTrue(result.DidConverge);
            Assert.IsTrue(Math.Abs(result.q[0] - 1) < 0.01f);
            Assert.IsTrue(Math.Abs(result.q[1] - 0) < 0.01f);
            Assert.IsTrue(Math.Abs(result.q[2] - 0) < 0.01f);
        }

        [TestMethod]
        public void LinearRobot_3()
        {

            var Robot = new RobotDynamics.Robots.Robot();
            Robot.AddLinearJoint(new Vector(0, 0, 0), new Vector(1, 0, 0))
                .AddLinearJoint(new Vector(0, 0, 0), new Vector(0, 1, 0))
                .AddLinearJoint(new Vector(0, 0, 0), new Vector(0, 0, 1));

            var result = Robot.ComputeInverseKinematics(new Vector(1, -1, 1), new RotationMatrix(Matrix.Eye(3).matrix), 0.001, 0.05, 100, 0.001f);
            Assert.IsTrue(result.DidConverge);
            Assert.IsTrue(Math.Abs(result.q[0] - 1) < 0.01f);
            Assert.IsTrue(Math.Abs(result.q[1] + 1) < 0.01f);
            Assert.IsTrue(Math.Abs(result.q[2] - 1) < 0.01f);
        }

        [TestMethod]
        public void MixedRobot()
        {
            var Robot = new RobotDynamics.Robots.Robot();
            Robot.AddJoint('x', new Vector(0, 4, 0))
                .AddJoint('x', new Vector(0, 4, 0))
                .AddLinearJoint(new Vector(0, 0, 0), new Vector(0, 0, 1));
            Robot.ComputeForwardKinematics(new double[] { 0, 0, 0 });
        }
        [TestMethod]
        public void FanucWithLinear()
        {
            var Robot = new FanucCR7();
            Robot.AddLinearJoint(new Vector(0, 0, 0), new Vector(0, 1, 0));
            Matrix alpha = Matrix.Eye(7) * 0.05;
            alpha.matrix[6, 6] = 1;
            var result = Robot.ComputeInverseKinematics(new Vector(0, 1100, 677), new RotationMatrix(Matrix.Eye(3).matrix), alpha, 0.001, 200);
            Assert.IsTrue(result.DidConverge);
            Assert.IsTrue(result.q[6] > 1);
        }
    }
}
