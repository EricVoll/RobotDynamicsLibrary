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
                type = JointType.Revolute
            };
        }

        public static Link Linear(Vector linearMotionDirection, Vector offset)
        {
            return new Link()
            {
                offset = offset,
                linearMotionDirection = linearMotionDirection / linearMotionDirection.Magnitude,
                type = JointType.Linear
            };
        }

        char axe;
        JointType type;
        Vector offset;
        Vector linearMotionDirection;

        double lastQ = -100;
        HomogenousTransformation lastHT = null;

        public HomogenousTransformation GetTransformation(double q)
        {
            if (lastQ == q) return lastHT;

            HomogenousTransformation HT;
            if (type == JointType.Revolute)
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
            if (axe == 'x') return new Vector(1, 0, 0);
            if (axe == 'y') return new Vector(0, 1, 0);
            if (axe == 'z') return new Vector(0, 0, 1);
            return null;
        }
    }


}