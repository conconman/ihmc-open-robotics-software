package us.ihmc.behaviors.continuousWalking;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.activeMapping.ContinuousPlanningParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.perception.HumanoidActivePerceptionModule;
import us.ihmc.perception.headless.TerrainPerceptionProcessWithDriver;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;

public class PerceptionBasedContinuousWalking
{
   private final ROS2SyncedRobotModel syncedRobot;
   private TerrainPerceptionProcessWithDriver perceptionTask;
   private HumanoidActivePerceptionModule activePerceptionModule;
   private final ContinuousPlanningParameters continuousPlanningParameters = new ContinuousPlanningParameters();

   public PerceptionBasedContinuousWalking(DRCRobotModel robotModel, String realsenseSerialNumber)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "nadia_terrain_perception_node");
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);
      perceptionTask = new TerrainPerceptionProcessWithDriver(realsenseSerialNumber,
                                                              robotModel.getSimpleRobotName(),
                                                              robotModel.getCollisionBoxProvider(),
                                                              robotModel.createFullRobotModel(),
                                                              RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ,
                                                              PerceptionAPI.D455_DEPTH_IMAGE,
                                                              PerceptionAPI.D455_COLOR_IMAGE,
                                                              PerceptionAPI.PERSPECTIVE_RAPID_REGIONS,
                                                              syncedRobot.getReferenceFrames(),
                                                              syncedRobot::update);
      activePerceptionModule = new HumanoidActivePerceptionModule(perceptionTask.getConfigurationParameters(), continuousPlanningParameters);
      activePerceptionModule.initializeContinuousElevationMappingTask(robotModel, ros2Node, syncedRobot.getReferenceFrames());

      perceptionTask.run();
      ThreadTools.sleepForever();
   }

   public void update()
   {
      activePerceptionModule.getContinuousMappingRemoteThread().setLatestHeightMapData(perceptionTask.getHumanoidPerceptionModule().getLatestHeightMapData());
   }
}
