using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobotDynamics.Controller
{
    public class JointsChangedEventArgs : EventArgs
    {

        public double[] joint_values { get; set; }
        public bool DidConverge { get; set; }

        public JointsChangedEventArgs(double[] joint_values, bool didConverge)
        {
            this.joint_values = joint_values;
            DidConverge = didConverge;
        }
    }
}
