package us.ihmc.humanoidBehaviors.tools;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomPlanarRegionHandler;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Acts as REA, reporting currently visible area as planar regions.
 */
public class SimulatedREAModule
{
   private volatile PlanarRegionsList map;

   private final IHMCROS2Publisher<PlanarRegionsListMessage> planarRegionPublisher;
   private final IHMCROS2Publisher<PlanarRegionsListMessage> realsenseSLAMPublisher;
   private RemoteSyncedHumanoidRobotState remoteSyncedHumanoidRobotState;

   private final HashMap<Integer, PlanarRegion> additionalPlanarRegions = new HashMap<>();
   private final PausablePeriodicThread thread;
   private MovingReferenceFrame neckFrame;
   private SimulatedDepthCamera simulatedDepthCamera;

   public SimulatedREAModule(PlanarRegionsList map, PubSubImplementation pubSubImplementation)
   {
      this(map, null, pubSubImplementation);
   }

   public SimulatedREAModule(PlanarRegionsList map, DRCRobotModel robotModel, PubSubImplementation pubSubImplementation)
   {
      this.map = map;

      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, ROS2Tools.REA.getNodeName());

      planarRegionPublisher = new IHMCROS2Publisher<>(ros2Node, PlanarRegionsListMessage.class, null, ROS2Tools.REA);
      realsenseSLAMPublisher = new IHMCROS2Publisher<>(ros2Node, PlanarRegionsListMessage.class, ROS2Tools.REALSENSE_SLAM_MAP_TOPIC_NAME);

      if (robotModel != null)
      {
         remoteSyncedHumanoidRobotState = new RemoteSyncedHumanoidRobotState(robotModel, ros2Node);
         neckFrame = remoteSyncedHumanoidRobotState.getHumanoidRobotState().getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH);
         double verticalFOV = 180.0; // TODO: Reduce FOV when behaviors support it better
         double horizontalFOV = 180.0;
         simulatedDepthCamera = new SimulatedDepthCamera(verticalFOV, horizontalFOV, neckFrame);
      }

      new ROS2Callback<>(ros2Node,
                         PlanarRegionsListMessage.class,
                         null,
                         ROS2Tools.REA.qualifyMore(ROS2Tools.REA_CUSTOM_REGION_QUALIFIER),
                         this::acceptAdditionalRegionList);

      thread = new PausablePeriodicThread(getClass().getSimpleName(), 0.5, this::process);
   }

   public void start()
   {
      thread.start();
   }

   public void stop()
   {
      thread.stop();
   }

   public void setMap(PlanarRegionsList map)
   {
      this.map = map;
   }

   private void process()
   {
      remoteSyncedHumanoidRobotState.pollHumanoidRobotState();
      ArrayList<PlanarRegion> combinedRegionsList = new ArrayList<>();
      if (remoteSyncedHumanoidRobotState != null)
      {
         if (remoteSyncedHumanoidRobotState.hasReceivedFirstMessage())
         {
            combinedRegionsList.addAll(simulatedDepthCamera.filterMapToVisible(map).getPlanarRegionsAsList());
         }
         else
         {
            // blank result
         }
      }
      else
      {
         combinedRegionsList.addAll(map.getPlanarRegionsAsList());
      }

      synchronized (this)
      {
         combinedRegionsList.addAll(additionalPlanarRegions.values());
         PlanarRegionsList combinedRegions = new PlanarRegionsList(combinedRegionsList);
         PlanarRegionsListMessage message = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(combinedRegions);
         planarRegionPublisher.publish(message);
         realsenseSLAMPublisher.publish(message);
      }
   }

   private void acceptAdditionalRegionList(PlanarRegionsListMessage message)
   {
      PlanarRegionsList newRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);

      synchronized (this)
      {
         for (PlanarRegion region : newRegions.getPlanarRegionsAsList())
         {
            if (region.getRegionId() == PlanarRegion.NO_REGION_ID)
            {
               continue;
            }
            else if (region.isEmpty())
            {
               additionalPlanarRegions.remove(region.getRegionId());
            }
            else
            {
               CustomPlanarRegionHandler.performConvexDecompositionIfNeeded(region);
               additionalPlanarRegions.put(region.getRegionId(), region);
            }
         }
      }
   }
}
