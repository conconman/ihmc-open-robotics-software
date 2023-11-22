package us.ihmc;

import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.avatar.colorVision.BlackflyImagePublisher;
import us.ihmc.avatar.colorVision.BlackflyImageRetriever;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.communication.ros2.ROS2HeartbeatMonitor;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.ouster.OusterDepthImagePublisher;
import us.ihmc.perception.ouster.OusterDepthImageRetriever;
import us.ihmc.perception.ouster.OusterNetServer;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.realsense.RealsenseDeviceManager;
import us.ihmc.perception.sceneGraph.arUco.ArUcoUpdateProcess;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeDetectionManager;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.perception.sensorHead.BlackflyLensProperties;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.RealsenseColorDepthImagePublisher;
import us.ihmc.sensors.RealsenseColorDepthImageRetriever;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;
import us.ihmc.tools.thread.RestartableThread;
import us.ihmc.tools.thread.RestartableThrottledThread;

import java.util.function.Supplier;

public class PerceptionAndAutonomyProcess
{
   private static final int ZED_CAMERA_ID = 0;
   private static final SideDependentList<ROS2Topic<ImageMessage>> ZED_COLOR_TOPICS = PerceptionAPI.ZED2_COLOR_IMAGES;
   private static final ROS2Topic<ImageMessage> ZED_DEPTH_TOPIC = PerceptionAPI.ZED2_DEPTH;

   private static final String REALSENSE_SERIAL_NUMBER = System.getProperty("d455.serial.number", "215122253249");
   private static final ROS2Topic<ImageMessage> REALSENSE_COLOR_TOPIC = PerceptionAPI.D455_COLOR_IMAGE;
   private static final ROS2Topic<ImageMessage> REALSENSE_DEPTH_TOPIC = PerceptionAPI.D455_DEPTH_IMAGE;

   private static final ROS2Topic<ImageMessage> OUSTER_DEPTH_TOPIC = PerceptionAPI.OUSTER_DEPTH_IMAGE;

   private static final String LEFT_BLACKFLY_SERIAL_NUMBER = System.getProperty("blackfly.left.serial.number", "00000000");
   private static final String RIGHT_BLACKFLY_SERIAL_NUMBER = System.getProperty("blackfly.right.serial.number", "17403057");
   private static final BlackflyLensProperties BLACKFLY_LENS = BlackflyLensProperties.BFS_U3_27S5C_FE185C086HA_1;
   private static final ROS2Topic<ImageMessage> BLACKFLY_IMAGE_TOPIC = PerceptionAPI.BLACKFLY_FISHEYE_COLOR_IMAGE.get(RobotSide.RIGHT);

   private static final double SCENE_GRAPH_UPDATE_FREQUENCY = 10.0;

   private final Supplier<ReferenceFrame> zedFrameSupplier;
   private final ROS2HeartbeatMonitor zedPointCloudHeartbeatMonitor;
   private final ROS2HeartbeatMonitor zedColorHeartbeatMonitor;
   private final ROS2HeartbeatMonitor zedDepthHeartbeatMonitor;
   private RawImage zedDepthImage;
   private final SideDependentList<RawImage> zedColorImages = new SideDependentList<>();
   private ZEDColorDepthImageRetriever zedImageRetriever;
   private ZEDColorDepthImagePublisher zedImagePublisher;
   private RestartableThread zedProcessAndPublishThread;

   private final Supplier<ReferenceFrame> realsenseFrameSupplier;
   private final ROS2HeartbeatMonitor realsenseHeartbeatMonitor;
   private RawImage realsenseDepthImage;
   private RawImage realsenseColorImage;
   private RealsenseColorDepthImageRetriever realsenseImageRetriever;
   private RealsenseColorDepthImagePublisher realsenseImagePublisher;
   private RestartableThread realsenseProcessAndPublishThread;

   private final Supplier<ReferenceFrame> ousterFrameSupplier;
   private final ROS2HeartbeatMonitor ousterDepthHeartbeatMonitor;
   private final ROS2HeartbeatMonitor lidarScanHeartbeatMonitor;
   private final ROS2HeartbeatMonitor heightMapHeartbeatMonitor;
   private OusterNetServer ouster;
   private RawImage ousterDepthImage;
   private OusterDepthImageRetriever ousterDepthImageRetriever;
   private OusterDepthImagePublisher ousterDepthImagePublisher;
   private RestartableThread ousterProcessAndPublishThread;

   private final SideDependentList<Supplier<ReferenceFrame>> blackflyFrameSuppliers = new SideDependentList<>();
   private final SideDependentList<ROS2HeartbeatMonitor> blackflyImageHeartbeatMonitors = new SideDependentList<>();
   private final SideDependentList<RawImage> blackflyImages = new SideDependentList<>();
   private final SideDependentList<BlackflyImageRetriever> blackflyImageRetrievers = new SideDependentList<>();
   private final SideDependentList<BlackflyImagePublisher> blackflyImagePublishers = new SideDependentList<>();
   private RestartableThread blackflyProcessAndPublishThread;

   private final Supplier<ReferenceFrame> robotPelvisFrameSupplier;
   private final ROS2SceneGraph sceneGraph;
   private RestartableThrottledThread sceneGraphUpdateThread;
   private final ROS2HeartbeatMonitor arUcoDetectionHeartbeatMonitor;
   private final ArUcoUpdateProcess arUcoUpdater;

   private final CenterposeDetectionManager centerposeDetectionManager;
   private final ROS2HeartbeatMonitor centerposeUpdateHeartbeatMonitor;

   // Sensor heartbeats to run main method without UI
   private ROS2Heartbeat zedHeartbeat;
   private ROS2Heartbeat realsenseHeartbeat;
   private ROS2Heartbeat ousterHeartbeat;
   private ROS2Heartbeat leftBlackflyHeartbeat;
   private ROS2Heartbeat rightBlackflyHeartbeat;

   public PerceptionAndAutonomyProcess(ROS2PublishSubscribeAPI ros2,
                                       Supplier<ReferenceFrame> zedFrameSupplier,
                                       Supplier<ReferenceFrame> realsenseFrameSupplier,
                                       Supplier<ReferenceFrame> ousterFrameSupplier,
                                       Supplier<ReferenceFrame> leftBlackflyFrameSupplier,
                                       Supplier<ReferenceFrame> rightBlackflyFrameSupplier,
                                       Supplier<ReferenceFrame> robotPelvisFrameSupplier,
                                       ReferenceFrame zed2iLeftCameraFrame)
   {
      this.zedFrameSupplier = zedFrameSupplier;
      zedPointCloudHeartbeatMonitor = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.REQUEST_ZED_POINT_CLOUD);
      zedColorHeartbeatMonitor = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.REQUEST_ZED_COLOR);
      zedDepthHeartbeatMonitor = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.REQUEST_ZED_DEPTH);
      initializeZEDHeartbeatCallbacks();

      this.realsenseFrameSupplier = realsenseFrameSupplier;
      realsenseHeartbeatMonitor = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.REQUEST_REALSENSE_POINT_CLOUD);
      initializeRealsenseHearbeatCallbacks();

      this.ousterFrameSupplier = ousterFrameSupplier;
      ousterDepthHeartbeatMonitor = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.REQUEST_OUSTER_DEPTH);
      lidarScanHeartbeatMonitor = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.REQUEST_LIDAR_SCAN);
      heightMapHeartbeatMonitor = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.REQUEST_HEIGHT_MAP);
      initializeOusterHeartbeatCallbacks();

      blackflyFrameSuppliers.put(RobotSide.LEFT, leftBlackflyFrameSupplier);
      blackflyFrameSuppliers.put(RobotSide.RIGHT, rightBlackflyFrameSupplier);
      blackflyImageHeartbeatMonitors.put(RobotSide.LEFT, new ROS2HeartbeatMonitor(ros2, PerceptionAPI.REQUEST_BLACKFLY_COLOR_IMAGE.get(RobotSide.LEFT)));
      blackflyImageHeartbeatMonitors.put(RobotSide.RIGHT, new ROS2HeartbeatMonitor(ros2, PerceptionAPI.REQUEST_BLACKFLY_COLOR_IMAGE.get(RobotSide.RIGHT)));
      initializeBlackflyHeartbeatCallbacks();

      this.robotPelvisFrameSupplier = robotPelvisFrameSupplier;
      sceneGraph = new ROS2SceneGraph(ros2);
      sceneGraphUpdateThread = new RestartableThrottledThread("SceneGraphUpdater", SCENE_GRAPH_UPDATE_FREQUENCY, this::updateSceneGraph);

      arUcoUpdater = new ArUcoUpdateProcess(sceneGraph, BLACKFLY_LENS, blackflyFrameSuppliers.get(RobotSide.RIGHT));
      arUcoDetectionHeartbeatMonitor = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.REQUEST_ARUCO);
      initializeArUcoHeartbeatCallbacks();

      centerposeDetectionManager = new CenterposeDetectionManager(ros2, zed2iLeftCameraFrame);
      centerposeUpdateHeartbeatMonitor = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.REQUEST_CENTERPOSE);
      initializeCenterposeHeartbeatCallbacks();

      sceneGraphUpdateThread.start(); // scene graph runs at all times
   }

   public void destroy()
   {
      LogTools.info("Destroying {}", this.getClass().getSimpleName());
      if (zedImageRetriever != null)
      {
         zedProcessAndPublishThread.stop();
         zedImagePublisher.destroy();
         zedImageRetriever.destroy();
      }

      if (realsenseImageRetriever != null)
      {
         realsenseProcessAndPublishThread.stop();
         realsenseImagePublisher.destroy();
         realsenseImageRetriever.destroy();
      }

      if (ouster != null)
      {
         System.out.println("Destroying OusterNetServer...");
         ouster.destroy();
         System.out.println("Destroyed OusterNetServer...");
         ousterProcessAndPublishThread.stop();
         ousterDepthImagePublisher.destroy();
         ousterDepthImageRetriever.destroy();
      }

      sceneGraphUpdateThread.stop();
      arUcoUpdater.stopArUcoDetection();

      if (blackflyProcessAndPublishThread != null)
         blackflyProcessAndPublishThread.stop();
      for (RobotSide side : RobotSide.values)
      {
         if (blackflyImagePublishers.get(side) != null)
            blackflyImagePublishers.get(side).destroy();
         if (blackflyImageRetrievers.get(side) != null)
            blackflyImageRetrievers.get(side).destroy();
      }
      LogTools.info("Destroyed {}", this.getClass().getSimpleName());

      if (zedHeartbeat != null)
         zedHeartbeat.destroy();
      if (realsenseHeartbeat != null)
         realsenseHeartbeat.destroy();
      if (ousterHeartbeat != null)
         ousterHeartbeat.destroy();
      if (leftBlackflyHeartbeat != null)
         leftBlackflyHeartbeat.destroy();
      if (rightBlackflyHeartbeat != null)
         rightBlackflyHeartbeat.destroy();
   }

   private void processAndPublishZED()
   {
      zedDepthImage = zedImageRetriever.getLatestRawDepthImage();
      for (RobotSide side : RobotSide.values)
      {
         zedColorImages.put(side, zedImageRetriever.getLatestRawColorImage(side));
      }

      // Do processing on image

      zedImagePublisher.setNextGpuDepthImage(zedDepthImage.get());
      for (RobotSide side : RobotSide.values)
      {
         zedImagePublisher.setNextColorImage(zedColorImages.get(side).get(), side);
      }

      zedDepthImage.release();
      zedColorImages.forEach(RawImage::release);
   }

   private void processAndPublishRealsense()
   {
      realsenseDepthImage = realsenseImageRetriever.getLatestRawDepthImage();
      realsenseColorImage = realsenseImageRetriever.getLatestRawColorImage();

      // Do processing on image

      realsenseImagePublisher.setNextDepthImage(realsenseDepthImage.get());
      realsenseImagePublisher.setNextColorImage(realsenseColorImage.get());

      realsenseDepthImage.release();
      realsenseColorImage.release();
   }

   private void processAndPublishOuster()
   {
      ousterDepthImage = ousterDepthImageRetriever.getLatestRawDepthImage();

      ousterDepthImagePublisher.setNextDepthImage(ousterDepthImage.get());

      ousterDepthImage.release();
   }

   private void processAndPublishBlackfly()
   {
      for (RobotSide side : RobotSide.values)
      {
         if (blackflyImageRetrievers.get(side) != null && blackflyImagePublishers.get(side) != null)
         {
            blackflyImages.put(side, blackflyImageRetrievers.get(side).getLatestRawImage());

            blackflyImagePublishers.get(side).setNextDistortedImage(blackflyImages.get(side).get());

            if (side == RobotSide.RIGHT && blackflyImages.get(RobotSide.RIGHT).getImageWidth() != 0)
               arUcoUpdater.setNextArUcoImage(blackflyImages.get(RobotSide.RIGHT).get());

            blackflyImages.get(side).release();
         }
      }
   }

   private void updateSceneGraph()
   {
      sceneGraph.updateSubscription();
      if (arUcoDetectionHeartbeatMonitor.isAlive() && arUcoUpdater.isInitialized())
         arUcoUpdater.updateArUcoDetection();
      if (centerposeUpdateHeartbeatMonitor.isAlive())
         centerposeDetectionManager.updateSceneGraph(sceneGraph);
      sceneGraph.updateOnRobotOnly(robotPelvisFrameSupplier.get());
      sceneGraph.updatePublication();
   }

   private void initializeZED()
   {
      zedImageRetriever = new ZEDColorDepthImageRetriever(ZED_CAMERA_ID, zedFrameSupplier);
      zedImagePublisher = new ZEDColorDepthImagePublisher(ZED_COLOR_TOPICS, ZED_DEPTH_TOPIC);
      zedProcessAndPublishThread = new RestartableThread("ZEDImageProcessAndPublish", this::processAndPublishZED);
      zedProcessAndPublishThread.start();
   }

   private void initializeZEDHeartbeatCallbacks()
   {
      zedPointCloudHeartbeatMonitor.setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            if (zedProcessAndPublishThread == null)
               initializeZED();
            zedImageRetriever.start();
            zedImagePublisher.startAll();
         }
         else
         {
            if (!zedColorHeartbeatMonitor.isAlive() && !zedDepthHeartbeatMonitor.isAlive())
               zedImageRetriever.stop();

            if (!zedColorHeartbeatMonitor.isAlive() && !centerposeUpdateHeartbeatMonitor.isAlive())
               zedImagePublisher.stopColor();

            if (!zedDepthHeartbeatMonitor.isAlive())
               zedImagePublisher.stopDepth();
         }
      });

      zedColorHeartbeatMonitor.setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            if (zedProcessAndPublishThread == null)
               initializeZED();
            zedImageRetriever.start();
            zedImagePublisher.startColor();
         }
         else
         {
            if (!zedPointCloudHeartbeatMonitor.isAlive() && !zedDepthHeartbeatMonitor.isAlive())
            {
               zedImageRetriever.stop();
               zedImagePublisher.stopAll();
            }
            else if (!zedPointCloudHeartbeatMonitor.isAlive() && !centerposeUpdateHeartbeatMonitor.isAlive())
            {
               zedImagePublisher.stopColor();
            }
         }
      });

      zedDepthHeartbeatMonitor.setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            if (zedProcessAndPublishThread == null)
               initializeZED();
            zedImageRetriever.start();
            zedImagePublisher.startDepth();
         }
         else
         {
            if (!zedPointCloudHeartbeatMonitor.isAlive() && !zedColorHeartbeatMonitor.isAlive() && !centerposeUpdateHeartbeatMonitor.isAlive())
            {
               zedImageRetriever.stop();
               zedImagePublisher.stopAll();
            }
            else if (!zedPointCloudHeartbeatMonitor.isAlive())
            {
               zedImagePublisher.stopDepth();
            }
         }
      });
   }

   private void initializeRealsense()
   {
      realsenseImageRetriever = new RealsenseColorDepthImageRetriever(new RealsenseDeviceManager(),
                                                                      REALSENSE_SERIAL_NUMBER,
                                                                      RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ,
                                                                      realsenseFrameSupplier);
      realsenseImagePublisher = new RealsenseColorDepthImagePublisher(REALSENSE_DEPTH_TOPIC, REALSENSE_COLOR_TOPIC);
      realsenseProcessAndPublishThread = new RestartableThread("RealsenseProcessAndPublish", this::processAndPublishRealsense);
      realsenseProcessAndPublishThread.start();
   }

   private void initializeRealsenseHearbeatCallbacks()
   {
      realsenseHeartbeatMonitor.setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            if (realsenseProcessAndPublishThread == null)
               initializeRealsense();
            realsenseImageRetriever.start();
            realsenseImagePublisher.startAll();
         }
         else
         {
            realsenseImagePublisher.stopAll();
            realsenseImageRetriever.stop();
         }
      });
   }

   private void initializeOuster()
   {
      ouster = new OusterNetServer();
      ousterDepthImageRetriever = new OusterDepthImageRetriever(ouster,
                                                                ousterFrameSupplier,
                                                                lidarScanHeartbeatMonitor::isAlive,
                                                                heightMapHeartbeatMonitor::isAlive);
      ousterDepthImagePublisher = new OusterDepthImagePublisher(ouster, OUSTER_DEPTH_TOPIC);
      ousterProcessAndPublishThread = new RestartableThread("OusterProcessAndPublish", this::processAndPublishOuster);
      ousterProcessAndPublishThread.start();
   }

   private void initializeOusterHeartbeatCallbacks()
   {
      ousterDepthHeartbeatMonitor.setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            if (ouster == null)
               initializeOuster();
            ouster.start();
            ousterDepthImageRetriever.start();
            ousterDepthImagePublisher.startDepth();
         }
         else
         {
            ouster.stop();
            ousterDepthImageRetriever.stop();
            ousterDepthImagePublisher.stopDepth();
         }
      });
   }

   private void initializeBlackfly(RobotSide side)
   {
      String serialNumber = side == RobotSide.LEFT ? LEFT_BLACKFLY_SERIAL_NUMBER : RIGHT_BLACKFLY_SERIAL_NUMBER;
      if (serialNumber.equals("00000000"))
      {
         LogTools.error("{} blackfly with serial number was not provided", side.getPascalCaseName());
         return;
      }

      blackflyImageRetrievers.put(side, new BlackflyImageRetriever(serialNumber, BLACKFLY_LENS, RobotSide.RIGHT, blackflyFrameSuppliers.get(side)));
      blackflyImagePublishers.put(side, new BlackflyImagePublisher(BLACKFLY_LENS, BLACKFLY_IMAGE_TOPIC));
      if (blackflyProcessAndPublishThread == null)
      {
         blackflyProcessAndPublishThread = new RestartableThread("BlackflyProcessAndPublish", this::processAndPublishBlackfly);
         blackflyProcessAndPublishThread.start();
      }
   }

   private void initializeBlackflyHeartbeatCallbacks()
   {
      blackflyImageHeartbeatMonitors.get(RobotSide.LEFT).setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            if (blackflyImageRetrievers.get(RobotSide.LEFT) == null)
               initializeBlackfly(RobotSide.LEFT);
            if (blackflyImageRetrievers.get(RobotSide.LEFT) != null && blackflyImagePublishers.get(RobotSide.LEFT) != null)
            {
               blackflyImageRetrievers.get(RobotSide.LEFT).start();
               blackflyImagePublishers.get(RobotSide.LEFT).startImagePublishing();
            }
         }
         else
         {
            if (blackflyImageRetrievers.get(RobotSide.LEFT) != null && blackflyImagePublishers.get(RobotSide.LEFT) != null)
            {
               blackflyImagePublishers.get(RobotSide.LEFT).stopImagePublishing();
               blackflyImageRetrievers.get(RobotSide.LEFT).stop();
            }
         }
      });

      blackflyImageHeartbeatMonitors.get(RobotSide.RIGHT).setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            if (blackflyImageRetrievers.get(RobotSide.RIGHT) == null)
               initializeBlackfly(RobotSide.RIGHT);

            if (blackflyImageRetrievers.get(RobotSide.RIGHT) != null && blackflyImagePublishers.get(RobotSide.RIGHT) != null)
            {
               blackflyImageRetrievers.get(RobotSide.RIGHT).start();
               blackflyImagePublishers.get(RobotSide.RIGHT).startImagePublishing();
            }
         }
         else
         {
            if (blackflyImageRetrievers.get(RobotSide.RIGHT) != null && blackflyImagePublishers.get(RobotSide.RIGHT) != null)
            {
               blackflyImagePublishers.get(RobotSide.RIGHT).stopImagePublishing();

               if (!arUcoDetectionHeartbeatMonitor.isAlive())
                  blackflyImageRetrievers.get(RobotSide.RIGHT).stop();
            }
         }
      });
   }

   private void initializeArUcoHeartbeatCallbacks()
   {
      arUcoDetectionHeartbeatMonitor.setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            if (blackflyImageRetrievers.get(RobotSide.RIGHT) == null)
               initializeBlackfly(RobotSide.RIGHT);

            if (blackflyImageRetrievers.get(RobotSide.RIGHT) != null)
            {
               blackflyImageRetrievers.get(RobotSide.RIGHT).start();
               arUcoUpdater.startArUcoDetection();
            }
         }
         else
         {
            if (blackflyImageRetrievers.get(RobotSide.RIGHT) != null)
            {
               arUcoUpdater.stopArUcoDetection();

               if (!blackflyImageHeartbeatMonitors.get(RobotSide.RIGHT).isAlive())
                  blackflyImageRetrievers.get(RobotSide.RIGHT).stop();
            }
         }
      });
   }

   private void initializeCenterposeHeartbeatCallbacks()
   {
      centerposeUpdateHeartbeatMonitor.setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            if (zedImageRetriever == null)
               initializeZED();

            zedImageRetriever.start();
            zedImagePublisher.startColor();
         }
         else
         {
            if (!zedPointCloudHeartbeatMonitor.isAlive() && !zedColorHeartbeatMonitor.isAlive() && !zedDepthHeartbeatMonitor.isAlive())
            {
               zedImagePublisher.stopColor();
               zedImageRetriever.stop();
            }
         }
      });
   }

   /*
    * This method is used to enable sensors without needing to run the UI.
    * Unneeded sensors can be commented out.
    */
   private void forceEnableAllSensors(ROS2PublishSubscribeAPI ros2)
   {
      zedHeartbeat = new ROS2Heartbeat(ros2, PerceptionAPI.REQUEST_ZED_POINT_CLOUD);
      zedHeartbeat.setAlive(true);

      realsenseHeartbeat = new ROS2Heartbeat(ros2, PerceptionAPI.REQUEST_REALSENSE_POINT_CLOUD);
      realsenseHeartbeat.setAlive(true);

      ousterHeartbeat = new ROS2Heartbeat(ros2, PerceptionAPI.REQUEST_OUSTER_DEPTH);
      ousterHeartbeat.setAlive(true);

      leftBlackflyHeartbeat = new ROS2Heartbeat(ros2, PerceptionAPI.REQUEST_BLACKFLY_COLOR_IMAGE.get(RobotSide.LEFT));
      leftBlackflyHeartbeat.setAlive(true);

      rightBlackflyHeartbeat = new ROS2Heartbeat(ros2, PerceptionAPI.REQUEST_BLACKFLY_COLOR_IMAGE.get(RobotSide.RIGHT));
      rightBlackflyHeartbeat.setAlive(true);
   }

   public static void main(String[] args)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "perception_autonomy_process");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      PerceptionAndAutonomyProcess perceptionAndAutonomyProcess = new PerceptionAndAutonomyProcess(ros2Helper,
                                                                                                   ReferenceFrame::getWorldFrame,
                                                                                                   ReferenceFrame::getWorldFrame,
                                                                                                   ReferenceFrame::getWorldFrame,
                                                                                                   ReferenceFrame::getWorldFrame,
                                                                                                   ReferenceFrame::getWorldFrame,
                                                                                                   ReferenceFrame::getWorldFrame,
                                                                                                   ReferenceFrame.getWorldFrame());

      // To run a sensor without the UI, uncomment the below line.
      // perceptionAndAutonomyProcess.forceEnableAllSensors(ros2Helper);
      Runtime.getRuntime().addShutdownHook(new Thread(perceptionAndAutonomyProcess::destroy, "PerceptionAutonomyProcessShutdown"));
   }
}