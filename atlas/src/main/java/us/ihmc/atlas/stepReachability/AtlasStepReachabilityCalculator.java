package us.ihmc.atlas.stepReachability;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasKinematicsCollisionModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.reachabilityMap.footstep.HumanoidStanceGenerator;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.physics.RobotCollisionModel;

public class AtlasStepReachabilityCalculator extends HumanoidStanceGenerator
{
   public AtlasStepReachabilityCalculator() throws Exception
   {
      super(new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS),
              "ihmc-open-robotics-software/atlas/src/main/resources");
   }

   public static void main(String[] args) throws Exception
   {
      new AtlasStepReachabilityCalculator();
   }

   @Override
   protected RobotCollisionModel getRobotCollisionModel(HumanoidJointNameMap jointMap)
   {
      AtlasKinematicsCollisionModel collisionModel = new AtlasKinematicsCollisionModel(jointMap);
      return collisionModel;
   }
}
