package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import java.util.Random;

import toolbox_msgs.msg.dds.WaypointBasedTrajectoryMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class WholeBodyTrajectoryToolboxMessageTools
{
   public static final Random random = new Random(1);

   public static interface FunctionTrajectory
   {
      public Pose3D compute(double time);
   }

   public static FunctionTrajectory createFunctionTrajectory(WaypointBasedTrajectoryMessage message)
   {
      return new FunctionTrajectory()
      {
         @Override
         public Pose3D compute(double time)
         {
            Pose3D current = new Pose3D();

            Pose3D previous = null;
            Pose3D next = null;
            double t0 = Double.NaN;
            double tf = Double.NaN;

            for (int i = 1; i < message.getWaypoints().size(); i++)
            {
               t0 = message.getWaypointTimes().get(i - 1);
               tf = message.getWaypointTimes().get(i);
               previous = message.getWaypoints().get(i - 1);
               next = message.getWaypoints().get(i);
               if (time < message.getWaypointTimes().get(i))
                  break;
            }

            double alpha = (time - t0) / (tf - t0);
            alpha = MathTools.clamp(alpha, 0.0, 1.0);
            current.interpolate(previous, next, alpha);

            return current;
         }
      };
   }

   public static WaypointBasedTrajectoryMessage createTrajectoryMessage(RigidBodyBasics endEffector, double t0, double tf, FunctionTrajectory trajectoryToDiscretize,
                                                                        SelectionMatrix6D selectionMatrix)
   {
      return createTrajectoryMessage(endEffector, t0, tf, 0.1, trajectoryToDiscretize, selectionMatrix);
   }

   public static WaypointBasedTrajectoryMessage createTrajectoryMessage(RigidBodyBasics endEffector, double t0, double tf, double timeResolution,
                                                                        FunctionTrajectory trajectoryToDiscretize, SelectionMatrix6D selectionMatrix)
   {
      int numberOfWaypoints = (int) Math.round((tf - t0) / timeResolution) + 1;
      timeResolution = (tf - t0) / (numberOfWaypoints - 1);

      double[] waypointTimes = new double[numberOfWaypoints];
      Pose3D[] waypoints = new Pose3D[numberOfWaypoints];

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double waypointTime = i * timeResolution + t0;

         waypointTimes[i] = waypointTime;
         waypoints[i] = trajectoryToDiscretize.compute(waypointTime);
      }

      return HumanoidMessageTools.createWaypointBasedTrajectoryMessage(endEffector, waypointTimes, waypoints, selectionMatrix);
   }

   public static double[] createDefaultExplorationAmplitudeArray(ConfigurationSpaceName... configurationSpaceNames)
   {
      double[] lowerLimit = new double[configurationSpaceNames.length];
      for (int i = 0; i < configurationSpaceNames.length; i++)
         lowerLimit[i] = configurationSpaceNames[i].getDefaultExplorationAmplitude();
      return lowerLimit;
   }

   public static double[] createDefaultExplorationUpperLimitArray(ConfigurationSpaceName... configurationSpaceNames)
   {
      double[] upperLimit = new double[configurationSpaceNames.length];
      for (int i = 0; i < configurationSpaceNames.length; i++)
         upperLimit[i] = configurationSpaceNames[i].getDefaultExplorationUpperLimit();
      return upperLimit;
   }

   public static double[] createDefaultExplorationLowerLimitArray(ConfigurationSpaceName... configurationSpaceNames)
   {
      double[] lowerLimit = new double[configurationSpaceNames.length];
      for (int i = 0; i < configurationSpaceNames.length; i++)
         lowerLimit[i] = configurationSpaceNames[i].getDefaultExplorationLowerLimit();
      return lowerLimit;
   }
}