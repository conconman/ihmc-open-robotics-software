package us.ihmc.valkyrieRosControl.upperBody;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HoldPositionControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.SmoothTransitionControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.StandPrepControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.rosControl.wholeRobot.*;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisher;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisherFactory;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.FloatingJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.tools.SettableTimestampProvider;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;
import us.ihmc.valkyrieRosControl.*;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.providers.LongProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.*;

import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.*;
import static us.ihmc.valkyrieRosControl.ValkyrieRosControlController.VALKYRIE_IHMC_ROS_CONTROLLER_NODE_NAME;

public class ValkyrieUpperBodyController extends IHMCWholeRobotControlJavaBridge
{
   private static final boolean ARM_MASS_SIMULATORS = true;

   private static final String[] torqueControlledJoints;
   private static final String[] positionControlledJoints = {"lowerNeckPitch", "neckYaw", "upperNeckPitch"};
   private static final String[] allValkyrieJoints;

   static
   {
      List<String> torqueControlledJointList = new ArrayList<>();

      // Torso joints
      torqueControlledJointList.add("torsoYaw");
      torqueControlledJointList.add("torsoPitch");
      torqueControlledJointList.add("torsoRoll");

      for (RobotSide robotSide : RobotSide.values)
      {
         // Upper arm joints
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "ShoulderPitch");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "ShoulderRoll");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "ShoulderYaw");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "ElbowPitch");

         if (ARM_MASS_SIMULATORS)
            continue;

         // Forearm joints
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "ForearmYaw");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "WristRoll");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "WristPitch");

         // Finger joints
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "IndexFingerMotorPitch1");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "MiddleFingerMotorPitch1");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "PinkyMotorPitch1");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "ThumbMotorRoll");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "ThumbMotorPitch1");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "ThumbMotorPitch2");
      }

      torqueControlledJoints = torqueControlledJointList.toArray(new String[0]);

      List<String> allJointsList = new ArrayList<>();
      Arrays.stream(torqueControlledJoints).forEach(allJointsList::add);
      Arrays.stream(positionControlledJoints).forEach(allJointsList::add);
      allValkyrieJoints = allJointsList.toArray(new String[0]);
   }

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

   private static final ValkyrieRobotVersion VERSION = ValkyrieRobotVersion.UPPER_BODY;
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, VERSION);

   private final RigidBodyBasics rootBody;
   private final OneDoFJointBasics[] controlledOneDoFJoints;

   private final ValkyrieUpperBodyStateEstimator stateEstimator;
   private ValkyrieUpperBodyOutputWriter outputWriter;
   private final RealtimeROS2Node ros2Node;
   private final RobotConfigurationDataPublisher robotConfigurationDataPublisher;

   private final YoDouble yoTime = new YoDouble("yoTime", registry);
   private final SettableTimestampProvider wallTimeProvider = new SettableTimestampProvider();
   private final TimestampProvider monotonicTimeProvider = RealtimeThread::getCurrentMonotonicClockTime;
   private final YoLowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList;

   private final HashMap<String, EffortJointHandle> effortJointHandles = new HashMap<>();
   private final HashMap<String, PositionJointHandle> positionJointHandles = new HashMap<>();
   private final HashMap<String, JointStateHandle> jointStateHandles = new HashMap<>();

   private YoVariableServer yoVariableServer;
   private final ValkyrieTorqueOffsetPrinter valkyrieTorqueOffsetPrinter = new ValkyrieTorqueOffsetPrinter();
   private StateMachine<HighLevelControllerName, HighLevelControllerState> controllerStateMachine;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;

   public ValkyrieUpperBodyController()
   {
      Pair<RigidBodyBasics, OneDoFJointBasics[]> upperBodySystem = ValkyrieUpperBodyController.createUpperBodySystem(robotModel);
      rootBody = upperBodySystem.getLeft();
      controlledOneDoFJoints = upperBodySystem.getRight();

      commandInputManager = new CommandInputManager(ControllerAPIDefinition.getControllerSupportedCommands());
      statusMessageOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());

      PeriodicRealtimeThreadSchedulerFactory realtimeThreadFactory = new PeriodicRealtimeThreadSchedulerFactory(ValkyriePriorityParameters.POSECOMMUNICATOR_PRIORITY);
      ros2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, realtimeThreadFactory, VALKYRIE_IHMC_ROS_CONTROLLER_NODE_NAME);
      stateEstimator = new ValkyrieUpperBodyStateEstimator(rootBody, controlledOneDoFJoints, registry);

      jointDesiredOutputList = new YoLowLevelOneDoFJointDesiredDataHolder(controlledOneDoFJoints, registry);

      robotConfigurationDataPublisher = createRobotConfigurationDataPublisher(robotModel,
                                                                              commandInputManager,
                                                                              statusMessageOutputManager,
                                                                              ros2Node,
                                                                              controlledOneDoFJoints,
                                                                              wallTimeProvider::getTimestamp,
                                                                              monotonicTimeProvider::getTimestamp);
      ros2Node.spin();
   }

   public static RobotConfigurationDataPublisher createRobotConfigurationDataPublisher(ValkyrieRobotModel robotModel,
                                                                                       CommandInputManager commandInputManager,
                                                                                       StatusMessageOutputManager statusMessageOutputManager,
                                                                                       RealtimeROS2Node ros2Node,
                                                                                       OneDoFJointBasics[] controlledOneDoFJoints,
                                                                                       LongProvider wallTimeProvider,
                                                                                       LongProvider monotonicTimeProvider)
   {
      final RobotConfigurationDataPublisher robotConfigurationDataPublisher;
      ROS2Topic<?> inputTopic = ROS2Tools.getControllerInputTopic(robotModel.getSimpleRobotName());
      ROS2Topic<?> outputTopic = ROS2Tools.getControllerOutputTopic(robotModel.getSimpleRobotName());
      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(inputTopic,
                                                                                                commandInputManager,
                                                                                                outputTopic,
                                                                                                statusMessageOutputManager,
                                                                                                ros2Node);
      controllerNetworkSubscriber.addMessageCollectors(ControllerAPIDefinition.createDefaultMessageIDExtractor(), 3);
      controllerNetworkSubscriber.addMessageValidator(ControllerAPIDefinition.createDefaultMessageValidation());

      RobotConfigurationDataPublisherFactory robotConfigurationDataPublisherFactory = new RobotConfigurationDataPublisherFactory();
      robotConfigurationDataPublisherFactory.setDefinitionsToPublish(controlledOneDoFJoints, null, null);
      robotConfigurationDataPublisherFactory.setROS2Info(ros2Node, outputTopic);

      Pose3D rootJointPosePlaceholder = new Pose3D();
      Twist rootJointTwistPlaceholder = new Twist();
      SpatialAcceleration rootJointAccelerationPlaceholder = new SpatialAcceleration();

      SensorTimestampHolder timestampHolder = new SensorTimestampHolder()
      {
         @Override
         public long getWallTime()
         {
            return wallTimeProvider.getValue();
         }

         @Override
         public long getSyncTimestamp()
         {
            return 0;
         }

         @Override
         public long getMonotonicTime()
         {
            return monotonicTimeProvider.getValue();
         }
      };

      FloatingJointStateReadOnly rootJointPlaceholder = new FloatingJointStateReadOnly()
      {
         @Override
         public Pose3DReadOnly getPose()
         {
            return rootJointPosePlaceholder;
         }

         @Override
         public TwistReadOnly getTwist()
         {
            return rootJointTwistPlaceholder;
         }

         @Override
         public SpatialAccelerationReadOnly getAcceleration()
         {
            return rootJointAccelerationPlaceholder;
         }
      };

      List<OneDoFJointStateReadOnly> jointSensorOutputs = new ArrayList<>();
      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         jointSensorOutputs.add(OneDoFJointStateReadOnly.createFromOneDoFJoint(controlledOneDoFJoints[i], true));
      }

      robotConfigurationDataPublisherFactory.setSensorSource(timestampHolder, rootJointPlaceholder, jointSensorOutputs, null, null);
      robotConfigurationDataPublisher = robotConfigurationDataPublisherFactory.createRobotConfigurationDataPublisher();
      return robotConfigurationDataPublisher;
   }

   @Override
   protected void init()
   {
      LogTools.info("Valkyrie robot version: " + VERSION);

      HashSet<String> torqueControlledJointsSet = new HashSet<>(Arrays.asList(torqueControlledJoints));
      HashSet<String> positionControlledJointsSet = new HashSet<>(Arrays.asList(positionControlledJoints));

      for (String jointName : allValkyrieJoints)
      {
         if (torqueControlledJointsSet.contains(jointName) && positionControlledJointsSet.contains(jointName))
         {
            throw new RuntimeException("Joint cannot be both position controlled and torque controlled via ROS Control! Joint name: " + jointName);
         }
         if (torqueControlledJointsSet.contains(jointName))
         {
            effortJointHandles.put(jointName, createEffortJointHandle(jointName));
         }
         else if (positionControlledJointsSet.contains(jointName))
         {
            positionJointHandles.put(jointName, createPositionJointHandle(jointName));
         }
         else
         {
            jointStateHandles.put(jointName, createJointStateHandle(jointName));
         }
      }

      HashMap<String, IMUHandle> imuHandles = new HashMap<>();
      String leftTrunkIMUSensor = ValkyrieSensorInformation.leftTrunkIMUSensor;
      imuHandles.put(leftTrunkIMUSensor, createIMUHandle(leftTrunkIMUSensor));

      // The upper body has no force-torque sensors
      HashMap<String, ForceTorqueSensorHandle> forceTorqueSensorHandles = new HashMap<>();

      ValkyrieSensorInformation sensorInformation = robotModel.getSensorInformation();
      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();

      ValkyrieUpperBodySensorReaderFactory sensorReaderFactory = new ValkyrieUpperBodySensorReaderFactory(wallTimeProvider,
                                                                                                          monotonicTimeProvider,
                                                                                                          stateEstimatorParameters,
                                                                                                          effortJointHandles,
                                                                                                          positionJointHandles,
                                                                                                          jointStateHandles,
                                                                                                          imuHandles,
                                                                                                          forceTorqueSensorHandles,
                                                                                                          robotModel.getJointMap(),
                                                                                                          sensorInformation);

      // TODO add IMU's
      sensorReaderFactory.build(rootBody, null, jointDesiredOutputList, registry);
      ValkyrieRosControlSensorReader sensorReader = sensorReaderFactory.getSensorReader();
      stateEstimator.init(sensorReader);

      // Setup upper body controller state machine
      StateMachineFactory<HighLevelControllerName, HighLevelControllerState> factory = new StateMachineFactory<>(HighLevelControllerName.class);
      factory.setNamePrefix("highLevelControllerName").setRegistry(registry).buildYoClock(yoTime);

      // Setup stand prep state
      StandPrepControllerState standPrepControllerState = new StandPrepControllerState(controlledOneDoFJoints,
                                                                                       robotModel.getHighLevelControllerParameters(),
                                                                                       jointDesiredOutputList,
                                                                                       yoTime);
      factory.addState(standPrepControllerState.getHighLevelControllerName(), standPrepControllerState);

      // Stand ready state
      HoldPositionControllerState standReadyState = new HoldPositionControllerState(HighLevelControllerName.STAND_READY,
                                                                                    controlledOneDoFJoints,
                                                                                    robotModel.getHighLevelControllerParameters(),
                                                                                    jointDesiredOutputList);
      factory.addState(standReadyState.getHighLevelControllerName(), standReadyState);

      // Setup manipulation state
      ValkyrieUpperBodyManipulationState manipulationState = new ValkyrieUpperBodyManipulationState(commandInputManager,
                                                                                                    robotModel.getControllerDT(),
                                                                                                    robotModel.getJointMap(),
                                                                                                    robotModel.getHighLevelControllerParameters(),
                                                                                                    robotModel.getWalkingControllerParameters(),
                                                                                                    rootBody,
                                                                                                    controlledOneDoFJoints,
                                                                                                    yoTime,
                                                                                                    graphicsListRegistry);
      factory.addState(manipulationState.getHighLevelControllerName(), manipulationState);

      // Smooth transition state
      SmoothTransitionControllerState standTransitionState = new SmoothTransitionControllerState("toWalking",
                                                                                                 HighLevelControllerName.STAND_TRANSITION_STATE,
                                                                                                 standReadyState,
                                                                                                 manipulationState,
                                                                                                 controlledOneDoFJoints,
                                                                                                 robotModel.getHighLevelControllerParameters());
      factory.addState(standTransitionState.getHighLevelControllerName(), standTransitionState);

      // Setup calibration state
      ValkyrieCalibrationControllerState calibrationControllerState = new ValkyrieCalibrationControllerState(null,
                                                                                                             null,
                                                                                                             controlledOneDoFJoints,
                                                                                                             yoTime,
                                                                                                             robotModel.getHighLevelControllerParameters(),
                                                                                                             jointDesiredOutputList,
                                                                                                             null,
                                                                                                             null,
                                                                                                             robotModel.getCalibrationParameters(),
                                                                                                             valkyrieTorqueOffsetPrinter);
      factory.addState(calibrationControllerState.getHighLevelControllerName(), calibrationControllerState);

      // Setup transitions
      YoEnum<HighLevelControllerName> requestedHighLevelControllerState = new YoEnum<>("requestedHighLevelControllerState", registry, HighLevelControllerName.class, true);

      // Stand prep -> Stand ready
      factory.addDoneTransition(standPrepControllerState.getHighLevelControllerName(), standReadyState.getHighLevelControllerName());

      // Calibration Request
      factory.addRequestedTransition(standPrepControllerState.getHighLevelControllerName(),     calibrationControllerState.getHighLevelControllerName(), requestedHighLevelControllerState);
      factory.addRequestedTransition(standReadyState.getHighLevelControllerName(),              calibrationControllerState.getHighLevelControllerName(), requestedHighLevelControllerState);

      // Calibration -> Stand prep
      factory.addDoneTransition(calibrationControllerState.getHighLevelControllerName(), standPrepControllerState.getHighLevelControllerName());

      // Stand ready -> Stand transition
      factory.addRequestedTransition(standReadyState.getHighLevelControllerName(), standTransitionState.getHighLevelControllerName(), requestedHighLevelControllerState);

      // Stand transition -> manipulation
      factory.addTransition(standTransitionState.getHighLevelControllerName(),
                            new StateTransition<>(manipulationState.getHighLevelControllerName(), new StateTransitionCondition()
                            {
                               @Override
                               public boolean testCondition(double timeInCurrentState)
                               {
                                  return standTransitionState.isDone(timeInCurrentState);
                               }

                               @Override
                               public boolean performOnEntry()
                               {
                                  return false;
                               }
                            }));

      // Build state machine
      controllerStateMachine = factory.build(robotModel.getHighLevelControllerParameters().getDefaultInitialControllerState());

      // Attach registries
      registry.addChild(standPrepControllerState.getYoRegistry());
      registry.addChild(standReadyState.getYoRegistry());
      registry.addChild(standTransitionState.getYoRegistry());
      registry.addChild(calibrationControllerState.getYoRegistry());
      registry.addChild(manipulationState.getYoRegistry());

      LogModelProvider logModelProvider = robotModel.getLogModelProvider();
      DataServerSettings logSettings = robotModel.getLogSettings();
      double estimatorDT = robotModel.getEstimatorDT();

      new DefaultParameterReader().readParametersInRegistry(registry);

      yoVariableServer = new YoVariableServer(getClass(), logModelProvider, logSettings, estimatorDT);
//      yoVariableServer.setMainRegistry(registry, fullRobotModel.getElevator(), graphicsListRegistry);
      yoVariableServer.start();

//      robotConfigurationDataPublisher.initialize();

      outputWriter = new ValkyrieUpperBodyOutputWriter(controlledOneDoFJoints, jointDesiredOutputList, effortJointHandles);
   }

   @Override
   protected void doControl(long rosTime, long duration)
   {
      wallTimeProvider.setTimestamp(rosTime);
      yoTime.set(Conversions.nanosecondsToSeconds(monotonicTimeProvider.getTimestamp()));

      /* Perform state estimation */
      stateEstimator.update();

      /* Run state machine */
      controllerStateMachine.doActionAndTransition();
      jointDesiredOutputList.overwriteWith(controllerStateMachine.getCurrentState().getOutputForLowLevelController());

      /* Write desireds */
      outputWriter.write();

      /* Update YoVariable server */
      yoVariableServer.update(monotonicTimeProvider.getTimestamp(), registry);
//      robotConfigurationDataPublisher.write();
   }

   private static void mutateRobotDefinition(RobotDefinition robotDefinition)
   {
      RigidBodyDefinition elevator = robotDefinition.getRootBodyDefinition();
      JointDefinition floatingJoint = elevator.getChildrenJoints().get(0);
      JointDefinition torsoYaw = floatingJoint.getSuccessor().getChildrenJoints().get(0);

      /* Replace floating joint with torso yaw joint */
      elevator.getChildrenJoints().clear();
      elevator.getChildrenJoints().add(torsoYaw);
      torsoYaw.setPredecessor(elevator);
   }

   public static Pair<RigidBodyBasics, OneDoFJointBasics[]> createUpperBodySystem(ValkyrieRobotModel robotModel)
   {
      /* Create un-mutated robot model to determine which joints are controllable using the method below */
      FullHumanoidRobotModel tmpFullHumanoidRobotModel = robotModel.createFullRobotModel();

      /* Mutate robot model to remove floating root joint */
      mutateRobotDefinition(robotModel.getRobotDefinition());

      /* Create multi-body system for upper body */
      RobotDefinition robotDefinition = robotModel.getRobotDefinition();
      RigidBodyBasics upperBodyRoot = robotDefinition.newInstance(ReferenceFrame.getWorldFrame());
      MultiBodySystemBasics upperBodySystem = MultiBodySystemBasics.toMultiBodySystemBasics(upperBodyRoot);

      /* Filter out unwanted joints */
      JointBasics[] tmpJointsToIgnore = AvatarControllerThread.createListOfJointsToIgnore(tmpFullHumanoidRobotModel, robotModel, robotModel.getSensorInformation());
      JointBasics[] tmpControlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(tmpFullHumanoidRobotModel, tmpJointsToIgnore);
      OneDoFJointBasics[] tmpJoints = MultiBodySystemTools.filterJoints(tmpControlledJoints, OneDoFJointBasics.class);

      OneDoFJointBasics[] controlledOneDoFJoints = new OneDoFJointBasics[tmpJoints.length];
      for (int i = 0; i < tmpJoints.length; i++)
      {
         controlledOneDoFJoints[i] = (OneDoFJointBasics) upperBodySystem.findJoint(tmpJoints[i].getName());
      }

      return Pair.of(upperBodyRoot, controlledOneDoFJoints);
   }

   public static void main(String[] args)
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRobotVersion.UPPER_BODY);

      HighLevelControllerParameters highLevelControllerParameters = robotModel.getHighLevelControllerParameters();
      List<GroupParameter<JointDesiredBehaviorReadOnly>> desiredJointBehaviors = highLevelControllerParameters.getDesiredJointBehaviors(STAND_PREP_STATE);

      if (desiredJointBehaviors != null)
      {
         for (int i = 0; i < desiredJointBehaviors.size(); i++)
         {
            System.out.println(desiredJointBehaviors.get(i).getGroupName());
            List<String> memberNames = desiredJointBehaviors.get(i).getMemberNames();
            for (int j = 0; j < memberNames.size(); j++)
            {
               System.out.println("  " + memberNames.get(j));
            }
         }
      }
   }
}
