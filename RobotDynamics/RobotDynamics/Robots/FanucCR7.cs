using RobotDynamics.MathUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobotDynamics.Robots
{
    public class FanucCR7 : Robot
    {
        public FanucCR7()
        {
            Robot Ro = new Robot()
                .AddJoint('z', new Vector(0, 0, 300))
                .AddJoint('x', new Vector(0, 50, 330))
                .AddJoint('x', new Vector(0, 0, 440))
                .AddJoint('y', new Vector(0, 100, 35))
                .AddJoint('x', new Vector(0, 320, 0))
                .AddJoint('y', new Vector(0, 80, 0));

            Links = Ro.Links;
        }
    }
}
