using RobotDynamics.MathUtilities;

namespace RobotDynamics.Robots
{
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