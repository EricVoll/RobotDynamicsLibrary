using RobotDynamics.MathUtilities;

namespace RobotDynamics.Robots
{
    public class Link
    {
        public enum JointType { Revolute, Linear }

        public Link()
        {

        }

        public static Link Revolute(char axe, Vector offset)
        {
            return new Link()
            {
                axe = axe,
                offset = offset,
                Type = JointType.Revolute
            };
        }

        public static Link Linear(Vector linearMotionDirection, Vector offset)
        {
            return new Link()
            {
                offset = offset,
                linearMotionDirection = linearMotionDirection / linearMotionDirection.Magnitude,
                Type = JointType.Linear
            };
        }

        char axe;
        public JointType Type { get; private set; }
        public Vector offset { get; private set; }
        public Vector linearMotionDirection { get; private set; }

        double lastQ = -100;
        HomogenousTransformation lastHT = null;

        public HomogenousTransformation GetTransformation(double q)
        {
            if (lastQ == q) return lastHT;

            HomogenousTransformation HT;
            if (Type == JointType.Revolute)
            {
                HT = new HomogenousTransformation(new RotationMatrix(q, axe), offset);
            }
            else
            {
                HT = new HomogenousTransformation(new RotationMatrix(Matrix.Eye(3).matrix), offset + q * linearMotionDirection);
            }

            lastHT = HT;
            return HT;
        } 

        public Vector GetN()
        {
            if (Type == JointType.Linear) return linearMotionDirection;

            if (axe == 'x') return new Vector(1, 0, 0);
            if (axe == 'y') return new Vector(0, 1, 0);
            if (axe == 'z') return new Vector(0, 0, 1);
            return null;
        }
    }


}