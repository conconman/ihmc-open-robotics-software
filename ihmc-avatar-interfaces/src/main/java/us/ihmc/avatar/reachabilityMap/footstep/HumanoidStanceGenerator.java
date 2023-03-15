package us.ihmc.avatar.reachabilityMap.footstep;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import org.jfree.util.LogTarget;
import toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule.KinematicsPlanningToolboxOptimizationSettings;
import us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule.SolutionQualityConvergenceDetector;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
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
      HAND_POSE, GENERATE_SINGLE_STANCE
   }

   private static final Mode mode = Mode.HAND_POSE;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final MaterialDefinition ghostMaterial = new MaterialDefinition(ColorDefinitions.Yellow().derive(0, 1, 1, 0.25));
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = simulationTestingParameters.getCreateGUI();

   static
   {
      simulationTestingParameters.setDataBufferSize(1 << 16);
   }

   private final CommandInputManager commandInputManager;
   private final YoRegistry mainRegistry;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final HumanoidKinematicsToolboxController toolboxController;
   private final KinematicsPlanningToolboxOptimizationSettings optimizationSettings;
   private SolutionQualityConvergenceDetector solutionQualityConvergenceDetector;

   private final YoBoolean initializationSucceeded;
   private final YoInteger numberOfIterations;
   private final YoDouble finalSolutionQuality;

   private SimulationConstructionSet scs;
   private BlockingSimulationRunner blockingSimulationRunner;

   private final HumanoidFloatingRootJointRobot robot;
   private final HumanoidFloatingRootJointRobot ghost;
   private final RobotController toolboxUpdater;

   public HumanoidStanceGenerator() throws Exception
   {
      mainRegistry = new YoRegistry("main");
      initializationSucceeded = new YoBoolean("initializationSucceeded", mainRegistry);
      numberOfIterations = new YoInteger("numberOfIterations", mainRegistry);
      finalSolutionQuality = new YoDouble("finalSolutionQuality", mainRegistry);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      optimizationSettings = new KinematicsPlanningToolboxOptimizationSettings();
      solutionQualityConvergenceDetector = new SolutionQualityConvergenceDetector(optimizationSettings, mainRegistry);

      DRCRobotModel robotModel = getRobotModel();
      DRCRobotModel ghostRobotModel = getRobotModel();

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

      robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      toolboxUpdater = createToolboxUpdater();
      robot.setController(toolboxUpdater);
      robot.setDynamic(false);
      robot.setGravity(0);

      // Green collision body
      addKinematicsCollisionGraphics(desiredFullRobotModel, robot, collisionModel);

      // Yellow initial body
      RobotDefinition robotDefinition = ghostRobotModel.getRobotDefinition();
      robotDefinition.setName("Ghost");
      RobotDefinitionTools.setRobotDefinitionMaterial(robotDefinition, ghostMaterial);
      ghost = ghostRobotModel.createHumanoidFloatingRootJointRobot(false);
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
         case HAND_POSE:
            testHandPose();
            break;

         default:
            throw new RuntimeException(mode + " is not implemented yet!");
      }

      ThreadTools.sleepForever();
   }

   protected abstract DRCRobotModel getRobotModel();

   protected abstract String getResourcesDirectory();

   protected void imposeJointLimitRestrictions(DRCRobotModel robotModel)
   {
   }

   protected abstract RobotCollisionModel getRobotCollisionModel(HumanoidJointNameMap jointMap);

   private void testHandPose() throws Exception
   {
      /* Pick random ground height and x, y, yaw for stance */
      double groundHeight = 0.1;
      Point2D stancePosition = new Point2D(0.2, -0.1);
      double stanceYaw = Math.toRadians(5.0);

      /* Create an object representing the robot's whole joint configuration when standing at that stance */
      FullHumanoidRobotModel initialFullRobotModel = createFullRobotModelAtInitialConfiguration(getRobotModel(), groundHeight, stancePosition, stanceYaw);

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
      CapturabilityBasedStatus capturabilityBasedStatus = createCapturabilityBasedStatus(initialFullRobotModel, getRobotModel(), true, true);
      toolboxController.updateCapturabilityBasedStatus(capturabilityBasedStatus);

      runKinematicsToolboxController();

      if (!initializationSucceeded.getBooleanValue())
      {
         throw new RuntimeException(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.");
      }
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

         if (linkGraphics != null)
            linkGraphics.combine(getGraphics(collidable));
      }
   }
}
