package us.ihmc.wholeBodyController.diagnostics;

import java.util.*;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.DiagnosticsWhenHangingHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.partNames.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.wholeBodyController.JointTorqueOffsetProcessor;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointTorqueOffsetEstimatorController implements RobotController, JointTorqueOffsetEstimator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private JointTorqueOffsetProcessor jointTorqueOffsetProcessor;

   private final MultiBodySystemBasics multiBodySystem;
   private final List<OneDoFJointBasics> oneDoFJoints;
   private final HumanoidJointNameMap jointNameMap;
   private final Map<String, OneDoFJointBasics> jointMap = new HashMap<>();

   private final LinkedHashMap<OneDoFJointBasics, PDController> pdControllers = new LinkedHashMap<OneDoFJointBasics, PDController>();
   private final LinkedHashMap<OneDoFJointBasics, YoDouble> desiredPositions = new LinkedHashMap<OneDoFJointBasics, YoDouble>();
   private final LinkedHashMap<OneDoFJointBasics, DiagnosticsWhenHangingHelper> helpers = new LinkedHashMap<OneDoFJointBasics, DiagnosticsWhenHangingHelper>();

   private final YoDouble ditherAmplitude = new YoDouble("ditherAmplitude", registry);
   private final YoDouble ditherFrequency = new YoDouble("ditherFrequency", registry);
   private final LinkedHashMap<OneDoFJointBasics, OneDoFJointDitherParameters> jointSpecificDitherParameters = new LinkedHashMap<>();

   private final YoDouble maximumTorqueOffset = new YoDouble("maximumTorqueOffset", registry);

   private final YoBoolean estimateTorqueOffset = new YoBoolean("estimateTorqueOffset", registry);
   private final YoBoolean transferTorqueOffsets = new YoBoolean("transferTorqueOffsets", registry);
   private final YoBoolean exportJointTorqueOffsetsToFile = new YoBoolean("recordTorqueOffsets", registry);

   private final TorqueOffsetPrinter torqueOffsetPrinter;

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final SideDependentList<YoPlaneContactState> footContactStates;

   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final YoBoolean hasReachedMaximumTorqueOffset = new YoBoolean("hasReachedMaximumTorqueOffset", registry);

   private final DoubleProvider currentTime;

   public JointTorqueOffsetEstimatorController(WholeBodySetpointParameters wholeBodySetpointParameters,
                                               HighLevelHumanoidControllerToolbox highLevelControllerToolbox,
                                               SideDependentList<YoPlaneContactState> footContactStates,
                                               BipedSupportPolygons bipedSupportPolygons,
                                               TorqueOffsetPrinter torqueOffsetPrinter,
                                               MultiBodySystemBasics multiBodySystem,
                                               HumanoidJointNameMap jointNameMap,
                                               DoubleProvider yoTime,
                                               JointTorqueOffsetEstimatorParameters parameters)
   {
      this.footContactStates = footContactStates;
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.controllerToolbox = highLevelControllerToolbox;
      this.torqueOffsetPrinter = torqueOffsetPrinter;
      this.multiBodySystem = multiBodySystem;
      this.currentTime = yoTime;
      this.jointNameMap = jointNameMap;

      ditherAmplitude.set(0.3);
      ditherFrequency.set(5.0);
      maximumTorqueOffset.set(15.0);

      estimateTorqueOffset.set(false);
      transferTorqueOffsets.set(false);

      oneDoFJoints = MultiBodySystemTools.filterJoints(multiBodySystem.getAllJoints(), OneDoFJointBasics.class);
      OneDoFJointBasics[] jointArray = oneDoFJoints.toArray(new OneDoFJointBasics[0]);

      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(jointArray);
      lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(jointArray, JointDesiredControlMode.EFFORT);

      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         jointMap.put(oneDoFJoints.get(i).getName(), oneDoFJoints.get(i));
      }

      createHelpers(true, parameters);

      for (int i = 0; i < this.oneDoFJoints.size(); i++)
      {
         OneDoFJointBasics joint = this.oneDoFJoints.get(i);

         String jointName = joint.getName();
         YoDouble desiredPosition = new YoDouble("q_d_calib_" + jointName, registry);
         desiredPosition.set(wholeBodySetpointParameters.getSetpoint(jointName));
         desiredPositions.put(joint, desiredPosition);

         jointSpecificDitherParameters.put(joint, new OneDoFJointDitherParameters(joint, registry));

         if (!hasTorqueOffsetForJoint(joint))
            continue;

         PDController controller = new PDController(jointName + "Calibration", registry);
         pdControllers.put(joint, controller);
      }

      setDefaultPDControllerGains();
   }

   public void setDither(double amplitude, double frequency)
   {
      ditherAmplitude.set(amplitude);
      ditherFrequency.set(frequency);
   }

   public void setJointDitherAmplitude(OneDoFJointBasics joint, double amplitude)
   {
      jointSpecificDitherParameters.get(joint).setAmplitude(amplitude);
   }
   
   public void setJointDitherPhase(OneDoFJointBasics joint, double phase)
   {
      jointSpecificDitherParameters.get(joint).setPhase(phase);
   }

   public void attachJointTorqueOffsetProcessor(JointTorqueOffsetProcessor jointTorqueOffsetProcessor)
   {
      this.jointTorqueOffsetProcessor = jointTorqueOffsetProcessor;
   }

   @Override
   public void initialize()
   {
      hasReachedMaximumTorqueOffset.set(false);
   }

   @Override
   public void doControl()
   {
      if (bipedSupportPolygons != null && footContactStates != null)
         bipedSupportPolygons.updateUsingContactStates(footContactStates);
      if (controllerToolbox != null)
         controllerToolbox.update();

      updateDiagnosticsWhenHangingHelpers();
      updatePDControllers();

      if (transferTorqueOffsets.getBooleanValue())
      {
         transferTorqueOffsets.set(false);
         transferTorqueOffsetsToOutputWriter();
      }

      if (exportJointTorqueOffsetsToFile.getBooleanValue() && torqueOffsetPrinter != null)
      {
         exportJointTorqueOffsetsToFile.set(false);
         exportTorqueOffsets();
      }

      for (int jointIndex = 0; jointIndex < oneDoFJoints.size(); jointIndex++)
      {
         OneDoFJointBasics joint = oneDoFJoints.get(jointIndex);
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointTorque(joint, joint.getTau());
      }

      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         OneDoFJointBasics joint = oneDoFJoints.get(i);
         if (!hasTorqueOffsetForJoint(joint))
         {
            lowLevelOneDoFJointDesiredDataHolder.setDesiredJointTorque(joint, 0.0);
            lowLevelOneDoFJointDesiredDataHolder.setDesiredJointPosition(joint, desiredPositions.get(joint).getValue());
            lowLevelOneDoFJointDesiredDataHolder.setDesiredJointVelocity(joint, 0.0);
         }
      }
   }

   public void updateDiagnosticsWhenHangingHelpers()
   {
      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = helpers.get(oneDoFJoints.get(i));
         if (diagnosticsWhenHangingHelper != null)
            diagnosticsWhenHangingHelper.update();
      }
   }

   private void updatePDControllers()
   {
      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         OneDoFJointBasics joint = oneDoFJoints.get(i);
         if (hasTorqueOffsetForJoint(joint))
            updatePDController(joint, currentTime.getValue());
      }
   }

   private void updatePDController(OneDoFJointBasics oneDoFJoint, double timeInCurrentState)
   {
      PDController pdController = pdControllers.get(oneDoFJoint);
      double desiredPosition = desiredPositions.get(oneDoFJoint).getDoubleValue();
      double desiredVelocity = 0.0;

      double tau = pdController.compute(oneDoFJoint.getQ(), desiredPosition, oneDoFJoint.getQd(), desiredVelocity);

      DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = helpers.get(oneDoFJoint);
      if (diagnosticsWhenHangingHelper != null)
      {
         tau = diagnosticsWhenHangingHelper.getTorqueToApply(tau, estimateTorqueOffset.getBooleanValue(), maximumTorqueOffset.getDoubleValue());
         if (hasReachedMaximumTorqueOffset.getBooleanValue()
               && Math.abs(diagnosticsWhenHangingHelper.getTorqueOffset()) == maximumTorqueOffset.getDoubleValue())
         {
            LogTools.warn("Reached maximum torque for at least one joint.");
            hasReachedMaximumTorqueOffset.set(true);
         }
      }

      OneDoFJointDitherParameters params = jointSpecificDitherParameters.get(oneDoFJoint);
      double amp = ditherAmplitude.getDoubleValue();
      if (params.hasAmplitude())
         amp = params.getAmplitude();
      double phase = 0.0;
      if (params.hasPhase())
         phase = Math.toRadians(params.getPhase());
      double ditherTorque = amp * Math.sin(2.0 * Math.PI * ditherFrequency.getDoubleValue() * timeInCurrentState + phase);
      oneDoFJoint.setTau(tau + ditherTorque);
   }

   private void createHelpers(boolean robotIsHanging, JointTorqueOffsetEstimatorParameters parameters)
   {
      if (parameters.hasArmJoints())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            for (ArmJointName armJointName : parameters.getArmJointsToRun())
            {
               boolean preserveY = armJointName.name().endsWith("PITCH");
               makeArmJointHelper(robotSide, preserveY, armJointName);
            }
         }
      }

      if (parameters.hasLegJoints() && parameters.hasSpineJoints())
      {
         SideDependentList<JointBasics> topLegJoints = new SideDependentList<JointBasics>();
         for (RobotSide robotSide : RobotSide.values)
            topLegJoints.set(robotSide, jointMap.get(jointNameMap.getLegJointName(robotSide, parameters.getLegJointsToRun()[0])));

         for (SpineJointName spineJointName : parameters.getSpineJointsToRun())
         {
            boolean preserveY = spineJointName.name().endsWith("PITCH");
            OneDoFJointBasics spineJoint = jointMap.get(jointNameMap.getSpineJointName(spineJointName));
            helpers.put(spineJoint, new DiagnosticsWhenHangingHelper(spineJoint, preserveY, robotIsHanging, topLegJoints, registry));
         }

      }

      if (parameters.hasLegJoints())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            for (LegJointName legJointName : parameters.getLegJointsToRun())
            {
               boolean preserveY = legJointName.name().endsWith("PITCH");
               makeLegJointHelper(robotSide, preserveY, legJointName);
            }
         }
      }
   }

   private void makeArmJointHelper(RobotSide robotSide, boolean preserveY, ArmJointName armJointName)
   {
      OneDoFJointBasics armJoint = jointMap.get(jointNameMap.getArmJointName(robotSide, armJointName));
      helpers.put(armJoint, new DiagnosticsWhenHangingHelper(armJoint, preserveY, registry));
   }

   private void makeLegJointHelper(RobotSide robotSide, boolean preserveY, LegJointName legJointName)
   {
      OneDoFJointBasics legJoint = jointMap.get(jointNameMap.getLegJointName(robotSide, legJointName));
      helpers.put(legJoint, new DiagnosticsWhenHangingHelper(legJoint, preserveY, registry));
   }

   private void setDefaultPDControllerGains()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         setPDControllerGains(jointMap.get(jointNameMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)), 30.0, 3.0);
         setPDControllerGains(jointMap.get(jointNameMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH)), 50.0, 5.0);
         setPDControllerGains(jointMap.get(jointNameMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)), 50.0, 5.0);
         setPDControllerGains(jointMap.get(jointNameMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)), 40.0, 4.0);
      }

      setPDControllerGains(jointMap.get(jointNameMap.getSpineJointName(SpineJointName.SPINE_YAW)), 30.0, 2.0);
      setPDControllerGains(jointMap.get(jointNameMap.getSpineJointName(SpineJointName.SPINE_PITCH)), 150.0, 8.0);
      setPDControllerGains(jointMap.get(jointNameMap.getSpineJointName(SpineJointName.SPINE_ROLL)), 150.0, 8.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         setPDControllerGains(jointMap.get(jointNameMap.getLegJointName(robotSide, LegJointName.HIP_YAW)), 30.0, 2.0);
         setPDControllerGains(jointMap.get(jointNameMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)), 150.0, 7.5);
         setPDControllerGains(jointMap.get(jointNameMap.getLegJointName(robotSide, LegJointName.HIP_ROLL)), 165, 6.0);
         setPDControllerGains(jointMap.get(jointNameMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)), 80.0, 3.0);
         setPDControllerGains(jointMap.get(jointNameMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH)), 20.0, 2.0);
         setPDControllerGains(jointMap.get(jointNameMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL)), 16.0, 1.0);
      }
   }

   private void setPDControllerGains(OneDoFJointBasics joint, double kp, double kd)
   {
      PDController pdController = pdControllers.get(joint);

      if (pdController != null)
      {
         pdController.setProportionalGain(kp);
         pdController.setDerivativeGain(kd);
      }
   }

   public void estimateTorqueOffset(boolean estimate)
   {
      estimateTorqueOffset.set(estimate);
   }

   public void exportTorqueOffsets()
   {
      if (torqueOffsetPrinter != null)
         torqueOffsetPrinter.printTorqueOffsets(this);
   }

   public void transferTorqueOffsetsToOutputWriter()
   {
      if (jointTorqueOffsetProcessor == null)
         return;

      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         DiagnosticsWhenHangingHelper helper = helpers.get(oneDoFJoints.get(i));

         if (helper != null)
         {
            double torqueOffset = helper.getTorqueOffset();
            jointTorqueOffsetProcessor.subtractTorqueOffset(oneDoFJoints.get(i), torqueOffset);
            helper.setTorqueOffset(0.0);
         }
      }
   }

   public double getJointCalibrationPosition(OneDoFJointBasics joint)
   {
      YoDouble yoDesiredPosition = desiredPositions.get(joint);
      if (yoDesiredPosition != null)
         return yoDesiredPosition.getValue();
      else
         return 0.0;
   }

   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   @Override
   public List<OneDoFJointBasics> getOneDoFJoints()
   {
      return oneDoFJoints;
   }

   @Override
   public double getEstimatedJointTorqueOffset(OneDoFJointBasics joint)
   {
      DiagnosticsWhenHangingHelper helper = helpers.get(joint);
      return helper == null ? 0.0 : helper.getTorqueOffset();
   }

   @Override
   public void resetEstimatedJointTorqueOffset(OneDoFJointBasics joint)
   {
      if (hasTorqueOffsetForJoint(joint))
         helpers.get(joint).setTorqueOffset(0.0);
   }

   @Override
   public boolean hasTorqueOffsetForJoint(OneDoFJointBasics joint)
   {
      return helpers.containsKey(joint);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return "Controller for estimating the joint torque offsets. It is based on " + DiagnosticsWhenHangingControllerState.class.getSimpleName() + ".";
   }

   private static class OneDoFJointDitherParameters
   {
      private final YoDouble amplitude;
      private final YoDouble phase;

      public OneDoFJointDitherParameters(OneDoFJointBasics joint, YoRegistry registry)
      {
         amplitude = new YoDouble(joint.getName() + "DitherAmplitude", registry);
         phase = new YoDouble(joint.getName() + "DitherPhase", registry);
         clear();
      }

      public void clear()
      {
         amplitude.setToNaN();
         phase.setToNaN();
      }

      public void setAmplitude(double amplitude)
      {
         this.amplitude.set(amplitude);
      }

      public void setPhase(double phase)
      {
         this.phase.set(phase);
      }

      public boolean hasAmplitude()
      {
         return !amplitude.isNaN();
      }

      public double getAmplitude()
      {
         return amplitude.getValue();
      }

      public boolean hasPhase()
      {
         return !phase.isNaN();
      }

      public double getPhase()
      {
         return phase.getValue();
      }
   }
}
