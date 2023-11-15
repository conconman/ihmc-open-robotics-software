package us.ihmc.perception.tools;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ActiveMappingTools
{
   public static void setRandomizedStraightGoalPoses(FramePose3D walkingStartPose,
                                                     SideDependentList<FramePose3D> stancePose,
                                                     SideDependentList<FramePose3D> goalPose,
                                                     float xDistance,
                                                     float zDistance)
   {
      float offsetX = (float) (Math.random() * 0.2 - 0.1);

      FramePose3D finalGoalMidPose = new FramePose3D();
      finalGoalMidPose.interpolate(stancePose.get(RobotSide.LEFT), stancePose.get(RobotSide.RIGHT), 0.5);

      for (RobotSide side : RobotSide.values)
      {
         RigidBodyTransform stanceToWalkingFrameTransform = new RigidBodyTransform();
         RigidBodyTransform worldToWalkingFrameTransform = new RigidBodyTransform();

         stanceToWalkingFrameTransform.set(finalGoalMidPose);
         worldToWalkingFrameTransform.set(walkingStartPose);
         worldToWalkingFrameTransform.invert();
         stanceToWalkingFrameTransform.multiply(worldToWalkingFrameTransform);

         double xWalkDistance = stanceToWalkingFrameTransform.getTranslation().norm();
         goalPose.get(side).getPosition().set(walkingStartPose.getPosition());
         goalPose.get(side).getOrientation().set(walkingStartPose.getOrientation());
         goalPose.get(side).appendTranslation(xWalkDistance + xDistance + offsetX, 0, finalGoalMidPose.getZ() + zDistance - walkingStartPose.getZ());
      }

      goalPose.get(RobotSide.LEFT).appendTranslation(0.0, 0.11, 0.0);
      goalPose.get(RobotSide.RIGHT).appendTranslation(0.0, -0.11, 0.0);
   }

   public static void setRandomGoalWithinBounds(Point2D generatedGoalLocation,
                                                SideDependentList<FramePose3D> startPose,
                                                SideDependentList<FramePose3D> goalPose,
                                                float xDistance,
                                                float offsetZ)
   {

      FramePose3D goalMidPose = new FramePose3D();
      goalMidPose.getTranslation().set(generatedGoalLocation.getX(), generatedGoalLocation.getY(), 0.0);

      // compute yaw as direction from start to goal
      double offsetYaw = Math.atan2(goalPose.get(RobotSide.LEFT).getY() - startPose.get(RobotSide.LEFT).getY(),
                                    goalPose.get(RobotSide.LEFT).getX() - startPose.get(RobotSide.LEFT).getX()) + Math.random() * 0.2 - 0.1;

      for (RobotSide side : RobotSide.values)
      {
         goalPose.get(side).getPosition().set(goalMidPose.getPosition());
         goalPose.get(side).getOrientation().setYawPitchRoll(offsetYaw, 0, 0);
      }

      goalPose.get(RobotSide.LEFT).appendTranslation(0.0, 0.11, 0.0);
      goalPose.get(RobotSide.RIGHT).appendTranslation(0.0, -0.11, 0.0);
   }

   public static Pose2D getNearestUnexploredNode(PlanarRegionsList planarRegionMap,
                                                 Point2DReadOnly gridOrigin,
                                                 Pose2D robotPose,
                                                 int gridSize,
                                                 float resolution)
   {
      Pose2D goalPose = new Pose2D(gridOrigin, 0.0);
      Point2D robotLocation = new Point2D(robotPose.getX(), robotPose.getY());
      float robotYaw = (float) robotPose.getYaw();
      float nearestDistance = Float.MAX_VALUE;
      for (int i = 0; i < gridSize; i++)
      {
         for (int j = 0; j < gridSize; j++)
         {
            Point3D point = new Point3D(gridOrigin.getX() + resolution * i, gridOrigin.getY() + resolution * j, 0.0);

            boolean explored = false;
            for (PlanarRegion region : planarRegionMap.getPlanarRegionsAsList())
            {
               Point3D projectedPoint = PlanarRegionTools.projectInZToPlanarRegion(point, region);
               explored |= PlanarRegionTools.isPointInWorldInsidePlanarRegion(region, projectedPoint);
            }

            if (!explored)
            {
               Point2D currentPoint = new Point2D(point.getX(), point.getY());
               float distanceToNode = (float) currentPoint.distance(robotLocation);
               float yawRobotToGoal = (float) Math.atan2(currentPoint.getY() - robotLocation.getY(), currentPoint.getX() - robotLocation.getX());

               if (distanceToNode < nearestDistance && distanceToNode > 0.3 && Math.abs(yawRobotToGoal - robotYaw) < Math.PI / 2.5f)
               {
                  goalPose.set(point.getX(), point.getY(), yawRobotToGoal);
                  nearestDistance = distanceToNode;
               }
            }
         }
      }

      return goalPose;
   }

   public static int getIndexFromCoordinates(double coordinate, float resolution, int offset)
   {
      return (int) (coordinate * resolution + offset);
   }

   public static double getCoordinateFromIndex(int index, double resolution, int offset)
   {
      return (index - offset) / resolution;
   }
}
