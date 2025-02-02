package us.ihmc.quadrupedRobotics.estimator.footSwitch;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.touchdownDetector.FootVelocityBasedTouchDownDetection;
import us.ihmc.commonWalkingControlModules.touchdownDetector.ForceBasedTouchDownDetection;
import us.ihmc.commonWalkingControlModules.touchdownDetector.JointTorqueBasedTouchdownDetector;
import us.ihmc.commonWalkingControlModules.touchdownDetector.TouchdownDetector;
import us.ihmc.commonWalkingControlModules.touchdownDetector.WeightedAverageWrenchCalculator;
import us.ihmc.commonWalkingControlModules.touchdownDetector.WrenchCalculator;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedRobotics.estimator.footSwitch.JointTorqueBasedWrenchCalculator.JointTorqueProvider;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.scs2.definition.robot.WrenchSensorDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.sensors.SimWrenchSensor;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class QuadrupedFootSwitchFactory
{
   private final RequiredFactoryField<Double> gravity = new RequiredFactoryField<>("gravity");
   private final RequiredFactoryField<YoRegistry> yoVariableRegistry = new RequiredFactoryField<>("yoVariableRegistry");
   private final RequiredFactoryField<QuadrantDependentList<ContactablePlaneBody>> footContactableBodies = new RequiredFactoryField<>( "footContactableBodies");
   private final RequiredFactoryField<FullQuadrupedRobotModel> fullRobotModel = new RequiredFactoryField<>("fullRobotModel");
   private final RequiredFactoryField<JointDesiredOutputListReadOnly> jointDesiredOutputList = new RequiredFactoryField<>("jointDesiredOutputList");
   private final RequiredFactoryField<FootSwitchType> footSwitchType = new RequiredFactoryField<>("footSwitchType");

   private final OptionalFactoryField<String> suffix = new OptionalFactoryField<>("suffix");

   private final OptionalFactoryField<QuadrantDependentList<Double>> kneeTorqueTouchdownThreshold = new OptionalFactoryField<>("kneeTorqueTouchdownThreshold");
   private final OptionalFactoryField<QuadrantDependentList<Double>> kneeTorqueForSureTouchdownThreshold = new OptionalFactoryField<>("kneeTorqueTouchdownThreshold");
   private final OptionalFactoryField<Boolean> useKneeTorqueTouchdown = new OptionalFactoryField<>("useKneeTorqueTouchdown");
   private final OptionalFactoryField<Boolean> useFootVelocityTouchdown = new OptionalFactoryField<>("useFootVelocityTouchdown");

   // Used to create the ground contact point based foot switches.
   private final OptionalFactoryField<Robot> simulatedRobot = new OptionalFactoryField<>("simulatedRobot");

   protected YoRegistry registry;

   private void setupTouchdownBasedFootSwitches(QuadrantDependentList<QuadrupedFootSwitchInterface> footSwitches, double totalRobotWeight)
   {

      DoubleParameter estimatedWrenchWeight = new DoubleParameter("estimatedWrenchAverageWeight" + suffix.get(), registry, 1.0);
      DoubleParameter desiredWrenchWeight = new DoubleParameter("desiredWrenchAverageWeight" + suffix.get(), registry, 0.0);

      DoubleParameter zForceThreshold = new DoubleParameter("ForceTchdwnDetectzForceThreshold" + suffix.get(), registry, 40.0);
      DoubleParameter zForceForSureThreshold = new DoubleParameter("ForceTchdwnDetectzForceForSureThreshold" + suffix.get(), registry, 450.0);

      DoubleProvider speedThreshold = null;
      DoubleProvider zVelocityThreshold = null;
      if (useFootVelocityTouchdown.get())
      {
         speedThreshold = new DoubleParameter("VelTchdwnDetectFootSpeedThreshold" + suffix.get(), registry, 0.8);
         zVelocityThreshold = new DoubleParameter("VelTchdwnDetectFootZVelocityThreshold" + suffix.get(), registry, 0.2);
      }
      IntegerProvider touchdownGlitchWindowSize = new IntegerParameter("touchdownGlitchWindowSize" + suffix.get(), registry, 10);


      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         List<JointTorqueProvider> estimatedJointTorqueProviders = new ArrayList<>();
         for (OneDoFJointBasics oneDoFJointBasics : fullRobotModel.get().getLegJointsList(robotQuadrant))
            estimatedJointTorqueProviders.add(new JointTorqueProvider()
            {
               @Override
               public double getTorque()
               {
                  return oneDoFJointBasics.getTau();
               }
            });

         List<JointTorqueProvider> desiredJointTorqueProviders = new ArrayList<>();
         for (OneDoFJointBasics oneDoFJointBasics : fullRobotModel.get().getLegJointsList(robotQuadrant))
            desiredJointTorqueProviders.add(new JointDesiredOutputTorqueProvider(jointDesiredOutputList.get().getJointDesiredOutput(oneDoFJointBasics)));

         ContactablePlaneBody contactableFoot = footContactableBodies.get().get(robotQuadrant);

         boolean dontDetectTouchdownIfAtJointLimit = true;


         WrenchCalculator estimatedTorqueBasedWrenchCalculator = new JointTorqueBasedWrenchCalculator(robotQuadrant.getShortName() + "estimated",
                                                                                                      fullRobotModel.get(), robotQuadrant,
                                                                                                      contactableFoot.getSoleFrame(), estimatedJointTorqueProviders);
         WrenchCalculator desiredTorqueBasedWrenchCalculator = new JointTorqueBasedWrenchCalculator(robotQuadrant.getShortName() + "desired",
                                                                                                    fullRobotModel.get(), robotQuadrant,
                                                                                                    contactableFoot.getSoleFrame(), desiredJointTorqueProviders);
         PairList<DoubleProvider, WrenchCalculator> wrenchCalculatorPairList = new PairList<>();
         wrenchCalculatorPairList.add(estimatedWrenchWeight, estimatedTorqueBasedWrenchCalculator);
         wrenchCalculatorPairList.add(desiredWrenchWeight, desiredTorqueBasedWrenchCalculator);

         WrenchCalculator weightedAverageWrenchCalculator = new WeightedAverageWrenchCalculator(robotQuadrant.getShortName(), suffix.get(), registry, wrenchCalculatorPairList);

         QuadrupedTouchdownDetectorBasedFootSwitch footSwitch = new QuadrupedTouchdownDetectorBasedFootSwitch(suffix.get(), robotQuadrant, contactableFoot,
                                                                                                              weightedAverageWrenchCalculator, touchdownGlitchWindowSize, totalRobotWeight,
                                                                                                              registry);


         TouchdownDetector forceBasedTouchDownDetection = new ForceBasedTouchDownDetection(suffix.get(), weightedAverageWrenchCalculator, robotQuadrant,
                                                                                           dontDetectTouchdownIfAtJointLimit, zForceThreshold, zForceForSureThreshold, registry);
         footSwitch.addTouchdownDetector(forceBasedTouchDownDetection);


         if (useKneeTorqueTouchdown.get())
         {
            JointTorqueBasedTouchdownDetector jointTorqueBasedTouchdownDetector = new JointTorqueBasedTouchdownDetector(suffix.get(), fullRobotModel.get().getLegJoint(robotQuadrant, LegJointName.KNEE_PITCH),
                                                                                      dontDetectTouchdownIfAtJointLimit, registry);
            jointTorqueBasedTouchdownDetector.setTorqueThreshold(kneeTorqueTouchdownThreshold.get().get(robotQuadrant));
            jointTorqueBasedTouchdownDetector.setTorqueForSureThreshold(kneeTorqueForSureTouchdownThreshold.get().get(robotQuadrant));

            footSwitch.addTouchdownDetector(jointTorqueBasedTouchdownDetector);
         }

         if (useFootVelocityTouchdown.get())
         {
            TouchdownDetector footVelocityBasedTouchDownDetection = new FootVelocityBasedTouchDownDetection(suffix.get(), fullRobotModel.get().getSoleFrame(robotQuadrant),
                                                                                                            robotQuadrant, speedThreshold, zVelocityThreshold, registry);
            footSwitch.addTouchdownDetector(footVelocityBasedTouchDownDetection);
         }

         footSwitches.set(robotQuadrant, footSwitch);
      }
   }

   private void setupWrenchBasedPointFootSwitches(QuadrantDependentList<QuadrupedFootSwitchInterface> footSwitches, double totalRobotWeight)
   {
      if (!simulatedRobot.hasValue())
      {
         LogTools.warn("simulatedRobot is null, creating touchdown based foot switches.");
         setupTouchdownBasedFootSwitches(footSwitches, totalRobotWeight);
         return;
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {  // this only works in simulation, as it's polling from the ground contact point
         ContactablePlaneBody contactablePlaneBody = footContactableBodies.get().get(robotQuadrant);
         JointBasics parentJoint = contactablePlaneBody.getRigidBody().getParentJoint();
         String jointName = parentJoint.getName();
         String forceSensorName = contactablePlaneBody.getName() + "ForceSensor";
         Robot robot = simulatedRobot.get();
         SimJointBasics forceTorqueSensorJoint = robot.findJoint(jointName);
         RigidBodyTransform transformToParentJoint = contactablePlaneBody.getSoleFrame().getTransformToDesiredFrame(parentJoint.getFrameAfterJoint());
         SimWrenchSensor wrenchSensor = forceTorqueSensorJoint.getAuxialiryData().addWrenchSensor(new WrenchSensorDefinition(forceSensorName, transformToParentJoint));
         WrenchCalculatorWrapper wrenchCalculatorWrapper = new WrenchCalculatorWrapper(wrenchSensor, contactablePlaneBody.getSoleFrame());
         QuadrupedFootSwitchInterface footSwitch = new QuadrupedWrenchBasedFootSwitch(wrenchCalculatorWrapper, contactablePlaneBody, totalRobotWeight, registry);
         footSwitches.set(robotQuadrant, footSwitch);
      }
   }

   private void setupSettableFootSwitches(QuadrantDependentList<QuadrupedFootSwitchInterface> footSwitches, double totalRobotWeight)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         QuadrupedSettableFootSwitch footSwitch = new QuadrupedSettableFootSwitch(footContactableBodies.get().get(robotQuadrant), totalRobotWeight, 4, registry);
         footSwitches.set(robotQuadrant, footSwitch);
      }
   }

   public QuadrantDependentList<QuadrupedFootSwitchInterface> createFootSwitches()
   {
      useKneeTorqueTouchdown.setDefaultValue(false);
      useFootVelocityTouchdown.setDefaultValue(false);
      suffix.setDefaultValue("");

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      registry = new YoRegistry("QuadrupedFootSwitchManagerRegistry" + suffix.get());

      yoVariableRegistry.get().addChild(registry);

      QuadrantDependentList<QuadrupedFootSwitchInterface> footSwitches = new QuadrantDependentList<>();
      double gravityMagnitude = Math.abs(gravity.get());
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.get().getElevator()) * gravityMagnitude;

      switch (footSwitchType.get())
      {
      case TouchdownBased:
         setupTouchdownBasedFootSwitches(footSwitches, totalRobotWeight);
         break;
      case WrenchBased:
         setupWrenchBasedPointFootSwitches(footSwitches, totalRobotWeight);
         break;
      default:
         setupSettableFootSwitches(footSwitches, totalRobotWeight);
      }

      FactoryTools.disposeFactory(this);

      return footSwitches;
   }

   // Factory setters
   public void setGravity(double gravity)
   {
      this.gravity.set(gravity);
   }

   public void setYoVariableRegistry(YoRegistry yoVariableRegistry)
   {
      this.yoVariableRegistry.set(yoVariableRegistry);
   }

   public void setFootContactableBodies(QuadrantDependentList<ContactablePlaneBody> footContactableBodies)
   {
      this.footContactableBodies.set(footContactableBodies);
   }

   public void setUseFootVelocityTouchdown(boolean useFootVelocityTouchdown)
   {
      this.useFootVelocityTouchdown.set(useFootVelocityTouchdown);
   }


   public void setUseKneeTorqueTouchdown(boolean useKneeTorqueTouchdown)
   {
      this.useKneeTorqueTouchdown.set(useKneeTorqueTouchdown);
   }

   public void setFullRobotModel(FullQuadrupedRobotModel fullRobotModel)
   {
      this.fullRobotModel.set(fullRobotModel);
   }

   public void setJointDesiredOutputList(JointDesiredOutputListReadOnly jointDesiredOutputList)
   {
      this.jointDesiredOutputList.set(jointDesiredOutputList);
   }

   public void setFootSwitchType(FootSwitchType footSwitchType)
   {
      this.footSwitchType.set(footSwitchType);
   }

   public void setKneeTouchdownThresholds(QuadrantDependentList<Double> kneeTouchdownThresholds)
   {
      this.kneeTorqueTouchdownThreshold.set(kneeTouchdownThresholds);
   }

   public void setKneeForSureTouchdownThresholds(QuadrantDependentList<Double> kneeTouchdownThresholds)
   {
      this.kneeTorqueForSureTouchdownThreshold.set(kneeTouchdownThresholds);
   }

   public void setSimulatedRobot(Robot simulatedRobot)
   {
      this.simulatedRobot.set(simulatedRobot);
   }

   public void setVariableSuffix(String suffix)
   {
      this.suffix.set(suffix);
   }

   private class JointDesiredOutputTorqueProvider implements JointTorqueProvider
   {
      private final JointDesiredOutputReadOnly jointDesiredOutput;

      public JointDesiredOutputTorqueProvider(JointDesiredOutputReadOnly jointDesiredOutputReadOnly)
      {
         this.jointDesiredOutput = jointDesiredOutputReadOnly;
      }

      @Override
      public double getTorque()
      {
         return jointDesiredOutput.getDesiredTorque();
      }
   }
}
