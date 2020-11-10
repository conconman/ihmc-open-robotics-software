package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.behaviors.scsSensorSimulation.SCSLidarAndCameraSimulator;
import us.ihmc.atlas.behaviors.tools.AtlasSimulationBasics;
import us.ihmc.avatar.environments.RealisticLabTerrainBuilder;
import us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.RemoteBehaviorInterface;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIRegistry;
import us.ihmc.humanoidBehaviors.ui.behaviors.LookAndStepBehaviorUI;
import us.ihmc.humanoidBehaviors.ui.simulation.EnvironmentInitialSetup;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;

import static us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments.TRIPLE_PLATFORM_HEIGHT;

public class AtlasLookAndStepBehaviorDemo extends AtlasSimulationBasics
{
   private static boolean RUN_REALSENSE_SLAM = Boolean.parseBoolean(System.getProperty("run.realsense.slam", "true"));
   private static boolean RUN_LIDAR_REA = Boolean.parseBoolean(System.getProperty("run.lidar.rea", "true"));
   private static boolean SHOW_REALSENSE_SLAM_UIS = Boolean.parseBoolean(System.getProperty("show.realsense.slam.uis"));

   {
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockAngledStepsUpAndDown,
                                                               0.0, 0.0, 0.0, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockAngledStepsUpAndDown,
                                                               0.0, Math.PI, 6.0, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockAngledStepsUpAndDown,
                                                               TRIPLE_PLATFORM_HEIGHT, Math.PI, 2.8, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockAngledStepsUpAndDown,
                                                               TRIPLE_PLATFORM_HEIGHT, 0.0,2.8, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockStepsUpAndDown,
                                                               0.0, 0.0, 0.0, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockStepsUpAndDown,
                                                               0.0, Math.PI, 6.0, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockStepsUpAndDown,
                                                               TRIPLE_PLATFORM_HEIGHT, Math.PI, 2.8, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockStepsUpAndDown,
                                                               TRIPLE_PLATFORM_HEIGHT, 0.0, 2.8, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateRealisticEasierStartingBlockRegions,
                                                               RealisticLabTerrainBuilder.PALLET_HEIGHT * 2.0, 0.0, 0.0, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockStepsUpAndDown,
                                                               0.0, 0.0, 0.0, 0.0));
      selectEnvironment();
   }

   private BehaviorUI behaviorUI;
   private final BehaviorModule behaviorModule;
   private AtlasPerceptionSimulation perceptionStack;
   private SCSLidarAndCameraSimulator lidarAndCameraSimulator;

   public AtlasLookAndStepBehaviorDemo()
   {

      JavaFXApplicationCreator.createAJavaFXApplication();

      ThreadTools.startAsDaemon(() -> perceptionStack = new AtlasPerceptionSimulation(COMMUNICATION_MODE_ROS2,
                                                                                      environmentInitialSetup.getPlanarRegionsSupplier().get(),
                                                                                      RUN_REALSENSE_SLAM,
                                                                                      SHOW_REALSENSE_SLAM_UIS,
                                                                                      RUN_LIDAR_REA,
                                                                                      createRobotModel()),
                                "PerceptionStack");
      ThreadTools.startAsDaemon(simulation, "Simulation");

      if (RUN_LIDAR_AND_CAMERA_SIMULATION)
         ThreadTools.startAsDaemon(this::lidarAndCameraSimulator, "LidarAndCamera");

      BehaviorUIRegistry behaviorRegistry = BehaviorUIRegistry.of(LookAndStepBehaviorUI.DEFINITION);

      behaviorModule = new BehaviorModule(behaviorRegistry, createRobotModel(), COMMUNICATION_MODE_ROS2, COMMUNICATION_MODE_KRYO);

      LogTools.info("Creating behavior user interface");
      Messager behaviorMessager;
      if (COMMUNICATION_MODE_KRYO == CommunicationMode.INTERPROCESS)
      {
         behaviorMessager = RemoteBehaviorInterface.createForUI(behaviorRegistry, "localhost");
      }
      else
      {
         behaviorMessager = behaviorModule.getMessager();
      }
      behaviorUI = new BehaviorUI(behaviorRegistry, behaviorMessager, createRobotModel(), COMMUNICATION_MODE_ROS2.getPubSubImplementation());
      behaviorUI.addOnCloseRequestListener(() -> ThreadTools.startAThread(() -> {
         destroy();
         Runtime.getRuntime().exit(0);
      }, "DestroyViaUI"));

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "DestroyViaKill"));
   }

   private void lidarAndCameraSimulator()
   {
      lidarAndCameraSimulator = new SCSLidarAndCameraSimulator(COMMUNICATION_MODE_ROS2.getPubSubImplementation(),
                                                               createCommonAvatarEnvironment(),
                                                               createRobotModel());
   }

   @Override
   protected boolean destroy()
   {
      boolean destroy = super.destroy();

      if (destroy)
      {
         behaviorUI.closeMessager();
         behaviorModule.destroy();
         perceptionStack.destroy();
         if (RUN_LIDAR_AND_CAMERA_SIMULATION)
            lidarAndCameraSimulator.destroy();
      }

      return destroy;
   }

   public static void main(String[] args)
   {
      AtlasSimulationBasics.runOrLogToFile(AtlasLookAndStepBehaviorDemo.class);
   }
}
