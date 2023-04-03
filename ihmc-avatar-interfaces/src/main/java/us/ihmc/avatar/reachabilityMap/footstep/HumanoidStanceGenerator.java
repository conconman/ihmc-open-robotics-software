package us.ihmc.avatar.reachabilityMap.footstep;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import toolbox_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule.KinematicsPlanningToolboxOptimizationSettings;
import us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule.SolutionQualityConvergenceDetector;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Pattern matched off of HumanoidKinematicsToolboxControllerTest. Check there for reference if needed.
 */
public abstract class HumanoidStanceGenerator
{
   private enum Mode
   {
      HAND_POSE, GENERATE_STANCE, STANCE_SWEEP
   }

   private static final Mode mode = Mode.GENERATE_STANCE;

   private static double xFootCenterOnOrigin = 0.0;
   private static double yFootCenterOnOrigin = 0.0;
   private static double footBodyFixedFrameHeight = 0.0;

   private static final RobotSide robotSide = RobotSide.LEFT;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final MaterialDefinition ghostMaterial = new MaterialDefinition(ColorDefinitions.Yellow().derive(0, 1, 1, 0.25));
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = simulationTestingParameters.getCreateGUI();

   static
   {
      simulationTestingParameters.setDataBufferSize(1 << 16);
   }

   private final DRCRobotModel robotModel;
   private final String resourcePath;

   private final CommandInputManager commandInputManager;
   private final YoRegistry mainRegistry;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final HumanoidKinematicsToolboxController toolboxController;
   private final KinematicsPlanningToolboxOptimizationSettings optimizationSettings;
   private final SolutionQualityConvergenceDetector solutionQualityConvergenceDetector;

   private final YoBoolean initializationSucceeded;
   private final YoInteger numberOfIterations;
   private final YoDouble finalSolutionQuality;

   private SimulationConstructionSet scs;
   private BlockingSimulationRunner blockingSimulationRunner;

   private final HumanoidFloatingRootJointRobot robot;
   private final HumanoidFloatingRootJointRobot ghost;
   private final RobotController toolboxUpdater;

   public HumanoidStanceGenerator(DRCRobotModel robotModelFromSubclass, String resourceString) throws Exception
   {
      this.robotModel = robotModelFromSubclass;
      this.resourcePath = resourceString;

      mainRegistry = new YoRegistry("main");
      initializationSucceeded = new YoBoolean("initializationSucceeded", mainRegistry);
      numberOfIterations = new YoInteger("numberOfIterations", mainRegistry);
      finalSolutionQuality = new YoDouble("finalSolutionQuality", mainRegistry);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      optimizationSettings = new KinematicsPlanningToolboxOptimizationSettings();
      solutionQualityConvergenceDetector = new SolutionQualityConvergenceDetector(optimizationSettings, mainRegistry);

      imposeJointLimitRestrictions(robotModel);

      FullHumanoidRobotModel desiredFullRobotModel = robotModel.createFullRobotModel();
      commandInputManager = new CommandInputManager(KinematicsToolboxModule.supportedCommands());

      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsToolboxModule.supportedStatus());

      double updateDT = 1.0e-3;
      toolboxController = new HumanoidKinematicsToolboxController(commandInputManager,
                                                                  statusOutputManager,
                                                                  desiredFullRobotModel,
                                                                  robotModel,
                                                                  updateDT,
                                                                  yoGraphicsListRegistry,
                                                                  mainRegistry);
      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(desiredFullRobotModel, toolboxController.getDesiredReferenceFrames()));
      toolboxController.setInitialRobotConfiguration(robotModel);

      RobotCollisionModel collisionModel = getRobotCollisionModel(robotModel.getJointMap());
      toolboxController.setCollisionModel(collisionModel);

      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(desiredFullRobotModel);
      referenceFrames.updateFrames();
      FramePoint3D footBodyFixedOriginInSoleFrame = new FramePoint3D(desiredFullRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame());
      footBodyFixedOriginInSoleFrame.changeFrame(referenceFrames.getSoleFrame(RobotSide.LEFT));
      footBodyFixedFrameHeight = footBodyFixedOriginInSoleFrame.getZ();

      robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      toolboxUpdater = createToolboxUpdater();
      robot.setController(toolboxUpdater);
      robot.setDynamic(false);
      robot.setGravity(0);

      // Green collision body
      addKinematicsCollisionGraphics(desiredFullRobotModel, robot, collisionModel);

      // Yellow initial body
      RobotDefinition robotDefinition = robotModel.getRobotDefinition();
      robotDefinition.setName("Ghost");
      RobotDefinitionTools.setRobotDefinitionMaterial(robotDefinition, ghostMaterial);
      ghost = robotModel.createHumanoidFloatingRootJointRobot(false);
      ghost.setDynamic(false);
      ghost.setGravity(0);
      hideGhost();

      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot[] {robot, ghost}, simulationTestingParameters);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
         scs.setCameraFix(0.0, 0.0, 1.0);
         scs.setCameraPosition(8.0, 0.0, 3.0);
         scs.startOnAThread();
         blockingSimulationRunner = new BlockingSimulationRunner(scs, 60.0 * 10.0);
      }

      switch (mode)
      {
         case HAND_POSE -> testHandPose();
         case GENERATE_STANCE ->
         {
            // This centers the stance foot at the origin
            centerStanceFoot();
            List<KinematicsToolboxOutputStatus> solutionList = new ArrayList<>();
            List<StanceConfiguration> stanceList = new ArrayList<>();

            /* Default standing */
            StanceConfiguration defaultStanding = new StanceConfiguration(0.0, 0.24, 0.0);
            stanceList.add(defaultStanding);
            generateFootStance(defaultStanding, solutionList);

            /* Wide stance */
            StanceConfiguration wideStance = new StanceConfiguration(0.0, 0.4, 0.0);
            stanceList.add(wideStance);
            generateFootStance(wideStance, solutionList);

            /* Nominal step with left foot forward */
            StanceConfiguration footForward = new StanceConfiguration(0.2, 0.24, 0.0);
            stanceList.add(footForward);
            generateFootStance(footForward, solutionList);

            /* Default stance width but left foot turned outward */
            StanceConfiguration turnedOutward = new StanceConfiguration(0.0, 0.24, 30.0);
            stanceList.add(turnedOutward);
            generateFootStance(turnedOutward, solutionList);

            /* Foot forward and turned inward */
            StanceConfiguration forwardAndTurnedInward = new StanceConfiguration(0.2, 0.24, -30.0);
            stanceList.add(forwardAndTurnedInward);
            generateFootStance(forwardAndTurnedInward, solutionList);

            /* Log all IK output solutions */
            setupJSONPathThenSave(solutionList, stanceList);
         }
         case STANCE_SWEEP ->
         {
            double minimumX = -0.3;
            double maximumX = 0.3;
            double minimumY = 0.1;
            double maximumY = 0.4;
            double minimumYaw = Math.toRadians(-45.0);
            double maximumYaw = Math.toRadians(45.0);

            double xIncrement = 0.1;
            double yIncrement = 0.1;
            double yawIncrement = Math.toRadians(15.0);

            List<KinematicsToolboxOutputStatus> solutionList = new ArrayList<>();
            List<StanceConfiguration> stanceList = new ArrayList<>();

            double epsilon = 1e-3;
            for (double x = minimumX; x <= maximumX + epsilon; x += xIncrement)
            {
               for (double y = minimumY; y <= maximumY + epsilon; y += yIncrement)
               {
                  for (double yaw = minimumYaw; yaw <= maximumYaw + epsilon; yaw += yawIncrement)
                  {
                     // call generate foot stance
                     // add stance to stanceList
                  }
               }
            }


         }
         default ->
         {
            throw new RuntimeException(mode + " is not implemented yet!");
         }
      }

      ThreadTools.sleepForever();
   }

   private void centerStanceFoot()
   {
      // Switch these positive and negative values to center the other foot
      if (RobotSide.RIGHT == robotSide)
      {
         xFootCenterOnOrigin = 0.0;
         yFootCenterOnOrigin = -0.12;
      }
      else
      {
         xFootCenterOnOrigin = -0.0;
         yFootCenterOnOrigin = 0.12;
      }
   }

   protected void imposeJointLimitRestrictions(DRCRobotModel robotModel)
   {
   }

   protected abstract RobotCollisionModel getRobotCollisionModel(HumanoidJointNameMap jointMap);

   private void testHandPose() throws Exception
   {
      /* Pick random ground height and x, y, yaw for stance */
      Point2D stancePosition = new Point2D(0, 0);
      double stanceYaw = Math.toRadians(5.0);

      /* Create an object representing the robot's whole joint configuration when standing at that stance */
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration(robotModel, 0.0, stancePosition, stanceYaw);

      /* Extract RobotConfigurationData, a ROS message used in the inverse kinematics */
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      /* Create an objective for where the left hand should be */
      KinematicsToolboxRigidBodyMessage leftHandObjective = new KinematicsToolboxRigidBodyMessage();
      /* Set the hash code of the left hand link, which is how the IK knows that this is a right hand objective */
      leftHandObjective.setEndEffectorHashCode(initialFullRobotModel.getHand(RobotSide.LEFT).hashCode());
      /* Desired left hand position */
      leftHandObjective.getDesiredPositionInWorld().set(0.5, 0.2, 0.9);
      /* Weight of the left hand objective */
      leftHandObjective.getLinearWeightMatrix().setXWeight(20.0);
      leftHandObjective.getLinearWeightMatrix().setYWeight(20.0);
      leftHandObjective.getLinearWeightMatrix().setZWeight(20.0);
      /* Tell the solver to ignore orientation of the left hand. So only position is controller */
      leftHandObjective.getAngularSelectionMatrix().setXSelected(false);
      leftHandObjective.getAngularSelectionMatrix().setYSelected(false);
      leftHandObjective.getAngularSelectionMatrix().setZSelected(false);
      /* Submit objective */
      commandInputManager.submitMessage(leftHandObjective);

      /* Create an objective for where the right hand should be */
      KinematicsToolboxRigidBodyMessage rightHandObjective = new KinematicsToolboxRigidBodyMessage();
      /* Set the hash code of the right hand link, which is how the IK knows that this is a right hand objective */
      rightHandObjective.setEndEffectorHashCode(initialFullRobotModel.getHand(RobotSide.RIGHT).hashCode());
      /* Desired right hand position */
      rightHandObjective.getDesiredPositionInWorld().set(0.4, -0.6, 1.3);
      /* Weight of the right hand objective */
      rightHandObjective.getLinearWeightMatrix().setXWeight(20.0);
      rightHandObjective.getLinearWeightMatrix().setYWeight(20.0);
      rightHandObjective.getLinearWeightMatrix().setZWeight(20.0);
      /* Tell the solver to ignore orientation of the right hand. So only position is controller */
      rightHandObjective.getAngularSelectionMatrix().setXSelected(false);
      rightHandObjective.getAngularSelectionMatrix().setYSelected(false);
      rightHandObjective.getAngularSelectionMatrix().setZSelected(false);
      /* Submit objective */
      commandInputManager.submitMessage(rightHandObjective);

      /* Create an objective for where the Center of Mass should be */
      KinematicsToolboxCenterOfMassMessage centerOfMassObjective = new KinematicsToolboxCenterOfMassMessage();
      /* Disable z. So only the x/y center of mass position are controller */
      centerOfMassObjective.getSelectionMatrix().setZSelected(false);
      /* Weight of the center of mass objective */
      centerOfMassObjective.getWeights().setXWeight(1.0);
      centerOfMassObjective.getWeights().setYWeight(1.0);
      /* Set the xy position of the desired center of mass to match the middle of the stance */
      centerOfMassObjective.getDesiredPositionInWorld().setX(stancePosition.getX());
      centerOfMassObjective.getDesiredPositionInWorld().setY(stancePosition.getY());

      /* Submit objective */
      commandInputManager.submitMessage(centerOfMassObjective);

      /* Various ways to configure the IK solver */
      KinematicsToolboxConfigurationMessage configurationMessage = new KinematicsToolboxConfigurationMessage();
      /* Disable the support region constraint */
      configurationMessage.setDisableSupportPolygonConstraint(true);
      /* IK solver will prevent self collisions */
      configurationMessage.setEnableCollisionAvoidance(true);
      /* Submit IK configurations */
      commandInputManager.submitMessage(configurationMessage);

      snapGhostToFullRobotModel(initialFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);

      /* This object, CapturabilityBasedStatus, specifies which feet are in contact. Here we say that both the left and right foot are in contact. The IK solver will in turn keep the feet locked in place. */
      CapturabilityBasedStatus capturabilityBasedStatus = createCapturabilityBasedStatus(initialFullRobotModel, robotModel, true, true);
      toolboxController.updateCapturabilityBasedStatus(capturabilityBasedStatus);

      runKinematicsToolboxController();

      if (!initializationSucceeded.getBooleanValue())
      {
         throw new RuntimeException(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
      }

      ThreadTools.sleepSeconds(4);
   }

   private void generateFootStance(StanceConfiguration stanceConfiguration, List<KinematicsToolboxOutputStatus> solutionList) throws Exception
   {
      /* Pick random ground height and x, y, yaw for stance */
      Point2D stancePosition = new Point2D(xFootCenterOnOrigin, yFootCenterOnOrigin);

      /* Create an object representing the robot's whole joint configuration when standing at that stance */
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration(robotModel, 0.0, stancePosition, 0.0);

      /* Extract RobotConfigurationData, a ROS message used in the inverse kinematics */
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      /* Create an objective for where the left foot should be */
      KinematicsToolboxRigidBodyMessage leftFootObjective = new KinematicsToolboxRigidBodyMessage();
      /* Set the hash code of the left foot link, which is how the IK knows that this is a right foot objective */
      leftFootObjective.setEndEffectorHashCode(initialFullRobotModel.getFoot(robotSide).hashCode());
      /* Desired left foot position */
      leftFootObjective.getDesiredPositionInWorld().set(stanceConfiguration.getX(), stanceConfiguration.getY(), footBodyFixedFrameHeight);
      leftFootObjective.getDesiredOrientationInWorld().setToYawOrientation(Math.toRadians(stanceConfiguration.getYaw()));

      /* Weight of the left foot objective */
      leftFootObjective.getLinearWeightMatrix().setXWeight(20.0);
      leftFootObjective.getLinearWeightMatrix().setYWeight(20.0);
      leftFootObjective.getLinearWeightMatrix().setZWeight(20.0);

      leftFootObjective.getAngularWeightMatrix().setXWeight(20.0);
      leftFootObjective.getAngularWeightMatrix().setYWeight(20.0);
      leftFootObjective.getAngularWeightMatrix().setZWeight(20.0);
      /* Submit objective */
      commandInputManager.submitMessage(leftFootObjective);

      /* Create an objective for where the right foot should be */
      KinematicsToolboxRigidBodyMessage rightFootObjective = new KinematicsToolboxRigidBodyMessage();
      /* Set the hash code of the right foot link, which is how the IK knows that this is a right hand objective */
      rightFootObjective.setEndEffectorHashCode(initialFullRobotModel.getFoot(robotSide.getOppositeSide()).hashCode());
      /* Desired right foot position */
      rightFootObjective.getDesiredPositionInWorld().set(0.0, 0.0, footBodyFixedFrameHeight);
      /* Weight of the right foot objective */
      rightFootObjective.getLinearWeightMatrix().setXWeight(20.0);
      rightFootObjective.getLinearWeightMatrix().setYWeight(20.0);
      rightFootObjective.getLinearWeightMatrix().setZWeight(20.0);

      rightFootObjective.getAngularWeightMatrix().setXWeight(20.0);
      rightFootObjective.getAngularWeightMatrix().setYWeight(20.0);
      rightFootObjective.getAngularWeightMatrix().setZWeight(20.0);
      /* Submit objective */
      commandInputManager.submitMessage(rightFootObjective);

      // TODO add objective for chest to have zero orientation to prevent leaning backward/forward

      /* Create an objective for where the Center of Mass should be */
      KinematicsToolboxCenterOfMassMessage centerOfMassObjective = new KinematicsToolboxCenterOfMassMessage();
      /* Disable z. So only the x/y center of mass position are controller */
      centerOfMassObjective.getSelectionMatrix().setZSelected(false);
      /* Weight of the center of mass objective */
      centerOfMassObjective.getWeights().setXWeight(1.0);
      centerOfMassObjective.getWeights().setYWeight(1.0);
      /* Set the xy position of the desired center of mass to match the middle of the stance */
      centerOfMassObjective.getDesiredPositionInWorld().setX(stancePosition.getX());
      centerOfMassObjective.getDesiredPositionInWorld().setY(stancePosition.getY());

      /* Submit objective */
      commandInputManager.submitMessage(centerOfMassObjective);

      /* Various ways to configure the IK solver */
      KinematicsToolboxConfigurationMessage configurationMessage = new KinematicsToolboxConfigurationMessage();
      /* Disable the support region constraint */
      configurationMessage.setDisableSupportPolygonConstraint(true);
      /* IK solver will prevent self collisions */
      configurationMessage.setEnableCollisionAvoidance(true);
      /* Submit IK configurations */
      commandInputManager.submitMessage(configurationMessage);

      snapGhostToFullRobotModel(initialFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);

      runKinematicsToolboxController();

      if (!initializationSucceeded.getBooleanValue())
      {
         throw new RuntimeException(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
      }

      solutionList.add(toolboxController.getSolution());
   }

   private void setupJSONPathThenSave(List<KinematicsToolboxOutputStatus> solutionList, List<StanceConfiguration> stanceConfigurations) throws Exception
   {
      DateTimeFormatter justGetDate = DateTimeFormatter.ofPattern("yyyy/MM/dd");
      LocalDateTime localDate = LocalDateTime.now();

      // Format filename
      String date = justGetDate.format(localDate).replace("/", "-");
      String rootUserPath = System.getProperty("user.home") + "/.ihmc/logs/";
      String fullFilePath = rootUserPath + date + "-HumanoidStanceGenerator.json";

      // Get JSON data
      saveStanceToJSONFile(fullFilePath, solutionList, stanceConfigurations);
   }

   private void saveStanceToJSONFile(String file, List<KinematicsToolboxOutputStatus> outputStatusList, List<StanceConfiguration> stanceConfigurations) throws Exception
   {
      FileTools.ensureFileExists(new File(file).toPath());

      JsonFactory jsonFactory = new JsonFactory();
      ObjectMapper objectMapper = new ObjectMapper(jsonFactory);
      ArrayNode rootNode = objectMapper.createArrayNode();

      JSONSerializer<KinematicsToolboxOutputStatus> serializer = new JSONSerializer<>(new KinematicsToolboxOutputStatusPubSubType());

      for (int i = 0; i < outputStatusList.size(); i++)
      {
         ObjectNode stanceAndSolution = objectMapper.createObjectNode();
         JsonNode outputStatusJSONNode = objectMapper.readTree(serializer.serializeToString(outputStatusList.get(i)));

         stanceAndSolution.put("x", stanceConfigurations.get(i).getX());
         stanceAndSolution.put("y", stanceConfigurations.get(i).getY());
         stanceAndSolution.put("yaw", stanceConfigurations.get(i).getYaw());
         stanceAndSolution.put("ik_solution", outputStatusJSONNode);

         rootNode.add(stanceAndSolution);
      }

      FileOutputStream outputStream = new FileOutputStream(file);
      PrintStream printStream = new PrintStream(outputStream);

      objectMapper.writerWithDefaultPrettyPrinter().writeValue(printStream, rootNode);

      outputStream.close();
      printStream.close();
   }

   private void runKinematicsToolboxController() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      initializationSucceeded.set(false);
      this.numberOfIterations.set(0);

      // Using convergence detector currently very slow
      // Can play around with convergence settings to speed things up
      solutionQualityConvergenceDetector.initialize();
      if (visualize)
      {
         while (!solutionQualityConvergenceDetector.isSolved())
         {
            blockingSimulationRunner.simulateNTicksAndBlockAndCatchExceptions(1);
            solutionQualityConvergenceDetector.submitSolutionQuality(toolboxController.getSolution().getSolutionQuality());
            solutionQualityConvergenceDetector.update();
         }
      }
      else
      {
         while (!solutionQualityConvergenceDetector.isSolved())
         {
            toolboxUpdater.doControl();
            solutionQualityConvergenceDetector.submitSolutionQuality(toolboxController.getSolution().getSolutionQuality());
            solutionQualityConvergenceDetector.update();
         }
      }
      finalSolutionQuality.set(toolboxController.getSolution().getSolutionQuality());
   }

   private RobotController createToolboxUpdater()
   {
      return new RobotController()
      {
         private final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(robot, toolboxController.getDesiredFullRobotModel());

         @Override
         public void doControl()
         {
            if (!initializationSucceeded.getBooleanValue())
            {
               initializationSucceeded.set(toolboxController.initialize());
               if (initializationSucceeded.getValue())
               { // Finish this tick so the robot state after initialization can be seen in SCS.
                  jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
                  return;
               }
            }

            if (initializationSucceeded.getBooleanValue())
            {
               toolboxController.updateInternal();
               jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
               numberOfIterations.increment();
            }
         }

         @Override
         public void initialize()
         {
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return mainRegistry;
         }

         @Override
         public String getName()
         {
            return mainRegistry.getName();
         }

         @Override
         public String getDescription()
         {
            return null;
         }
      };
   }

   private void hideGhost()
   {
      ghost.setPositionInWorld(new Point3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
   }

   private void snapGhostToFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      new JointAnglesWriter(ghost, fullHumanoidRobotModel).updateRobotConfigurationBasedOnFullRobotModel();
   }

   public static FullHumanoidRobotModel createFullRobotModelAtInitialConfiguration(DRCRobotModel robotModel,
                                                                                   double groundHeight,
                                                                                   Tuple2DReadOnly offset,
                                                                                   double offsetYaw)
   {
      FullHumanoidRobotModel initialFullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      robotModel.getDefaultRobotInitialSetup(groundHeight, offsetYaw).initializeRobot(robot);
      DRCPerfectSensorReaderFactory drcPerfectSensorReaderFactory = new DRCPerfectSensorReaderFactory(robot, 0);
      drcPerfectSensorReaderFactory.build(initialFullRobotModel.getRootJoint(), null, null, null, null);
      SensorDataContext sensorDataContext = new SensorDataContext();
      long timestamp = drcPerfectSensorReaderFactory.getSensorReader().read(sensorDataContext);
      drcPerfectSensorReaderFactory.getSensorReader().compute(timestamp, sensorDataContext);
      initialFullRobotModel.getRootJoint().getJointPose().prependTranslation(offset.getX(), offset.getY(), 0.0);
      initialFullRobotModel.updateFrames();
      return initialFullRobotModel;
   }

   public static SideDependentList<ContactablePlaneBody> extractContactableFeet(FullHumanoidRobotModel robotModel,
                                                                                RobotContactPointParameters<RobotSide> contactPointParameters)
   {
      ContactableBodiesFactory<RobotSide> factory = new ContactableBodiesFactory<>();
      factory.setFullRobotModel(robotModel);
      factory.setReferenceFrames(new HumanoidReferenceFrames(robotModel));
      factory.setFootContactPoints(contactPointParameters.getControllerFootGroundContactPoints());
      return new SideDependentList<>(factory.createFootContactablePlaneBodies());
   }

   public static RobotConfigurationData extractRobotConfigurationData(FullHumanoidRobotModel fullRobotModel)
   {
      fullRobotModel.updateFrames();
      OneDoFJointBasics[] joints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
      RobotConfigurationData robotConfigurationData = RobotConfigurationDataFactory.create(joints, forceSensorDefinitions, imuDefinitions);
      RobotConfigurationDataFactory.packJointState(robotConfigurationData, Arrays.stream(joints).collect(Collectors.toList()));
      robotConfigurationData.getRootPosition().set(fullRobotModel.getRootJoint().getJointPose().getPosition());
      robotConfigurationData.getRootOrientation().set(fullRobotModel.getRootJoint().getJointPose().getOrientation());
      return robotConfigurationData;
   }

   public static CapturabilityBasedStatus createCapturabilityBasedStatus(FullHumanoidRobotModel currentRobotModel,
                                                                         DRCRobotModel drcRobotModel,
                                                                         boolean isLeftFootInSupport,
                                                                         boolean isRightFootInSupport)
   {
      return createCapturabilityBasedStatus(currentRobotModel, drcRobotModel.getContactPointParameters(), isLeftFootInSupport, isRightFootInSupport);
   }

   public static CapturabilityBasedStatus createCapturabilityBasedStatus(FullHumanoidRobotModel currentRobotModel,
                                                                         RobotContactPointParameters<RobotSide> contactPointParameters,
                                                                         boolean isLeftFootInSupport,
                                                                         boolean isRightFootInSupport)
   {
      CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

      SideDependentList<ContactablePlaneBody> contactableFeet = extractContactableFeet(currentRobotModel, contactPointParameters);

      IDLSequence.Object<Point3D> leftFootSupportPolygon2d = capturabilityBasedStatus.getLeftFootSupportPolygon3d();
      IDLSequence.Object<Point3D> rightFootSupportPolygon2d = capturabilityBasedStatus.getRightFootSupportPolygon3d();
      if (isLeftFootInSupport)
         contactableFeet.get(RobotSide.LEFT)
                        .getContactPointsCopy()
                        .stream()
                        .peek(cp -> cp.changeFrame(worldFrame))
                        .forEach(cp -> leftFootSupportPolygon2d.add().set(cp.getX(), cp.getY(), 0.0));
      if (isRightFootInSupport)
         contactableFeet.get(RobotSide.RIGHT)
                        .getContactPointsCopy()
                        .stream()
                        .peek(cp -> cp.changeFrame(worldFrame))
                        .forEach(cp -> rightFootSupportPolygon2d.add().set(cp.getX(), cp.getY(), 0.0));
      return capturabilityBasedStatus;
   }

   private static Graphics3DObject getGraphics(Collidable collidable)
   {
      Shape3DReadOnly shape = collidable.getShape();
      RigidBodyTransform transformToParentJoint = collidable.getShape()
                                                            .getReferenceFrame()
                                                            .getTransformToDesiredFrame(collidable.getRigidBody().getParentJoint().getFrameAfterJoint());
      Graphics3DObject graphics = new Graphics3DObject();
      graphics.transform(transformToParentJoint);
      AppearanceDefinition appearance = YoAppearance.DarkGreen();
      appearance.setTransparency(0.5);

      if (shape instanceof Sphere3DReadOnly)
      {
         Sphere3DReadOnly sphere = (Sphere3DReadOnly) shape;
         graphics.translate(sphere.getPosition());
         graphics.addSphere(sphere.getRadius(), appearance);
      }
      else if (shape instanceof Capsule3DReadOnly)
      {
         Capsule3DReadOnly capsule = (Capsule3DReadOnly) shape;
         RigidBodyTransform transform = new RigidBodyTransform();
         EuclidGeometryTools.orientation3DFromZUpToVector3D(capsule.getAxis(), transform.getRotation());
         transform.getTranslation().set(capsule.getPosition());
         graphics.transform(transform);
         graphics.addCapsule(capsule.getRadius(), capsule.getLength() + 2.0 * capsule.getRadius(), // the 2nd term is removed internally.
                             appearance);
      }
      else if (shape instanceof Box3DReadOnly)
      {
         Box3DReadOnly box = (Box3DReadOnly) shape;
         graphics.translate(box.getPosition());
         graphics.rotate(new RotationMatrix(box.getOrientation()));
         graphics.addCube(box.getSizeX(), box.getSizeY(), box.getSizeZ(), true, appearance);
      }
      else if (shape instanceof PointShape3DReadOnly)
      {
         PointShape3DReadOnly pointShape = (PointShape3DReadOnly) shape;
         graphics.translate(pointShape);
         graphics.addSphere(0.01, appearance);
      }
      else
      {
         LogTools.info("Unsupported shape: " + shape.getClass().getSimpleName());
      }
      return graphics;
   }

   public static void addKinematicsCollisionGraphics(FullHumanoidRobotModel fullRobotModel, Robot robot, RobotCollisionModel collisionModel)
   {
      List<Collidable> robotCollidables = collisionModel.getRobotCollidables(fullRobotModel.getElevator());

      for (Collidable collidable : robotCollidables)
      {
         Graphics3DObject linkGraphics = robot.getLink(collidable.getRigidBody().getName()).getLinkGraphics();

//         if (linkGraphics != null)
//            linkGraphics.combine(getGraphics(collidable));
      }
   }

   private static class StanceConfiguration
   {
      private final double x;
      private final double y;
      private final double yaw;

      public StanceConfiguration(double x, double y, double yaw)
      {
         this.x = x;
         this.y = y;
         this.yaw = yaw;
      }

      public double getX()
      {
         return x;
      }

      public double getY()
      {
         return y;
      }

      public double getYaw()
      {
         return yaw;
      }
   }
}
