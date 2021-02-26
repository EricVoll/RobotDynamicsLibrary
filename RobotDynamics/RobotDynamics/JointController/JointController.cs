using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using static RobotDynamics.Robots.Link;

namespace RobotDynamics.Controller
{
    /// <summary>
    /// A file used to control the robot joints
    /// </summary>
    public class JointController
    {
        public JointController(float kp, float tolerance, int DOF, JointType[] types)
        {
            State = new RobotState();
            this.kp = kp;
            this.tolerance = tolerance;
            State.CurrentJointValues = new double[DOF];
            State.CurrentTargetJointValues = new double[DOF];
            jointTypes = types;
        }

        public EventHandler<JointsChangedEventArgs> jointsChangedEvent;

        private float kp;
        private float tolerance;
        private JointType[] jointTypes;

        private RobotState State { get; set; }

        /// <summary>
        /// Reports the new joint values to the controller state
        /// </summary>
        /// <param name="q"></param>
        public void ReportNewTargetJointValues(double[] q, bool didConverge)
        {
            for (int i = 0; i < q.Length; i++)
            {
                if (jointTypes[i] == JointType.Revolute)
                    q[i] %= Math.PI * 2;
            }
            State.CurrentTargetJointValues = q;
            State.LastIterationDidConverge = didConverge;
        }

        /// <summary>
        /// Report a new frame to the controller. This will invoke an update in all controllers, if necessary
        /// </summary>
        /// <param name="timeSinceLastFrame">The time since the last update in seconds</param>
        public void ReportNewFrame(float timeSinceLastFrame)
        {
            if (State.JointValuesNeedUpdating)
            {
                PerformPControllerStep(timeSinceLastFrame, kp);
                State.UpdateJointValueUpdatingState(tolerance);
                jointsChangedEvent?.Invoke(this, new JointsChangedEventArgs(State.CurrentJointValues, State.LastIterationDidConverge));
            }
        }

        /// <summary>
        /// Updates the State with the corresponding controller values
        /// </summary>
        /// <param name="deltaTime">The time since the last update in seconds</param>
        /// <param name="kp">Proportional controller parameter</param>
        private void PerformPControllerStep(float deltaTime, float kp)
        {
            for (int i = 0; i < State.CurrentTargetJointValues.Length; i++)
            {
                State.CurrentJointValues[i] = State.CurrentJointValues[i] + (State.CurrentTargetJointValues[i] - State.CurrentJointValues[i]) * kp * deltaTime;
            }
        }

        public class RobotState
        {
            private double[] currentTargetJointValues;

            public bool JointValuesNeedUpdating { get; set; }
            public double[] CurrentJointValues { get; set; }
            public double[] CurrentTargetJointValues
            {
                get => currentTargetJointValues;
                set
                {
                    JointValuesNeedUpdating = true;
                    currentTargetJointValues = value;
                }
            }

            public bool LastIterationDidConverge { get; internal set; }

            /// <summary>
            /// Returns true if all angles are within the given tolerance
            /// </summary>
            /// <param name="tol"></param>
            /// <returns></returns>
            public bool UpdateJointValueUpdatingState(double tol)
            {
                for (int i = 0; i < CurrentTargetJointValues.Length; i++)
                {
                    if (Math.Abs(CurrentJointValues[i] - CurrentTargetJointValues[i]) > tol)
                    {
                        return false;
                    }
                }
                return true;
            }
        }
    }
}
