package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states;

import org.apache.commons.math3.util.Precision;
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryBalanceManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.TouchdownErrorCompensator;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class RecoveringSwingState extends PushRecoveryState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final int additionalFootstepsToConsider;
   private final Footstep nextFootstep = new Footstep();
   private final FootstepTiming footstepTiming = new FootstepTiming();
   private double swingTime;

   private final Footstep[] footsteps;
   private final FootstepTiming[] footstepTimings;

   private final FramePose3D actualFootPoseInWorld = new FramePose3D(worldFrame);
   private final FramePose3D desiredFootPoseInWorld = new FramePose3D(worldFrame);

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final CenterOfMassHeightManager comHeightManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;
   private final LegConfigurationManager legConfigurationManager;

   private final YoDouble fractionOfSwingToStraightenSwingLeg = new YoDouble("fractionOfSwingToStraightenSwingLeg", registry);
   private final YoDouble fractionOfSwingToCollapseStanceLeg = new YoDouble("fractionOfSwingToCollapseStanceLeg", registry);

   private final YoDouble remainingSwingTimeAccordingToPlan = new YoDouble("remainingSwingTimeAccordingToPlan", registry);
   private final YoDouble estimatedRemainingSwingTimeUnderDisturbance = new YoDouble("estimatedRemainingSwingTimeUnderDisturbance", registry);
   private final YoDouble optimizedRemainingSwingTime = new YoDouble("optimizedRemainingSwingTime", registry);
   private final YoDouble icpErrorThresholdToSpeedUpSwing = new YoDouble("icpErrorThresholdToSpeedUpSwing", registry);

   private final YoBoolean finishSingleSupportWhenICPPlannerIsDone = new YoBoolean("finishSingleSupportWhenICPPlannerIsDone", registry);
   private final BooleanProvider minimizeAngularMomentumRateZDuringSwing;

   private final FrameQuaternion tempOrientation = new FrameQuaternion();
   private final FrameVector3D tempAngularVelocity = new FrameVector3D();

   private final TouchdownErrorCompensator touchdownErrorCompensator;

   protected final RobotSide swingSide;
   protected final RobotSide supportSide;

   private final YoBoolean hasMinimumTimePassed = new YoBoolean("hasMinimumTimePassed", registry);
   private final YoDouble minimumSwingFraction = new YoDouble("minimumSwingFraction", registry);

   private final WalkingMessageHandler walkingMessageHandler;
   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final FullHumanoidRobotModel fullRobotModel;

   private final PushRecoveryBalanceManager balanceManager;

   public RecoveringSwingState(PushRecoveryStateEnum stateEnum,
                               WalkingMessageHandler walkingMessageHandler,
                               TouchdownErrorCompensator touchdownErrorCompensator,
                               HighLevelHumanoidControllerToolbox controllerToolbox,
                               PushRecoveryControlManagerFactory managerFactory,
                               PushRecoveryControllerParameters pushRecoveryControllerParameters,
                               WalkingFailureDetectionControlModule failureDetectionControlModule,
                               YoRegistry parentRegistry)
   {
      super(stateEnum, parentRegistry);

      this.supportSide = stateEnum.getSupportSide();
      swingSide = supportSide.getOppositeSide();

      minimumSwingFraction.set(0.5);

      this.walkingMessageHandler = walkingMessageHandler;
      footSwitches = controllerToolbox.getFootSwitches();
      fullRobotModel = controllerToolbox.getFullRobotModel();

      this.balanceManager = managerFactory.getOrCreateBalanceManager();
      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();

      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.touchdownErrorCompensator = touchdownErrorCompensator;

      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
      legConfigurationManager = managerFactory.getOrCreateLegConfigurationManager();

      fractionOfSwingToStraightenSwingLeg.set(pushRecoveryControllerParameters.getLegConfigurationParameters().getFractionOfSwingToStraightenLeg());
      fractionOfSwingToCollapseStanceLeg.set(pushRecoveryControllerParameters.getLegConfigurationParameters().getFractionOfSwingToCollapseStanceLeg());

//      finishSingleSupportWhenICPPlannerIsDone.set(pushRecoveryControllerParameters.finishSingleSupportWhenICPPlannerIsDone());
      minimizeAngularMomentumRateZDuringSwing = new BooleanParameter("minimizeAngularMomentumRateZDuringSwing", registry,
              pushRecoveryControllerParameters.minimizeAngularMomentumRateZDuringSwing());


      additionalFootstepsToConsider = balanceManager.getMaxNumberOfStepsToConsider();
      footsteps = Footstep.createFootsteps(additionalFootstepsToConsider);
      footstepTimings = FootstepTiming.createTimings(additionalFootstepsToConsider);

      setYoVariablesToNaN();
   }

   private void setYoVariablesToNaN()
   {
      optimizedRemainingSwingTime.setToNaN();
      estimatedRemainingSwingTimeUnderDisturbance.setToNaN();
      remainingSwingTimeAccordingToPlan.setToNaN();
   }

   public RobotSide getSwingSide()
   {
      return swingSide;
   }

   @Override
   public RobotSide getSupportSide()
   {
      return supportSide;
   }


   @Override
   public void doAction(double timeInState)
   {
      balanceManager.setSwingFootTrajectory(swingSide, feetManager.getSwingTrajectory(swingSide));
      balanceManager.computeICPPlan();

      boolean requestSwingSpeedUp = balanceManager.getICPErrorMagnitude() > icpErrorThresholdToSpeedUpSwing.getDoubleValue();

      if (walkingMessageHandler.hasRequestedFootstepAdjustment())
      {
         boolean footstepHasBeenAdjusted = walkingMessageHandler.pollRequestedFootstepAdjustment(nextFootstep);

         if (footstepHasBeenAdjusted)
         {
            walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
            failureDetectionControlModule.setNextFootstep(nextFootstep);
            updateFootstepParameters();

            feetManager.adjustSwingTrajectory(swingSide, nextFootstep, swingTime);

            balanceManager.adjustFootstep(nextFootstep);
            balanceManager.computeICPPlan();

            updateHeightManager();
         }
      }
      else
      {
         boolean footstepIsBeingAdjusted = balanceManager.checkAndUpdateStepAdjustment(nextFootstep);

         if (footstepIsBeingAdjusted)
         {
            requestSwingSpeedUp = true;
            walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
            failureDetectionControlModule.setNextFootstep(nextFootstep);
            updateFootstepParameters();

            feetManager.adjustSwingTrajectory(swingSide, nextFootstep, swingTime);

            balanceManager.adjustFootstep(nextFootstep);
            balanceManager.computeICPPlan();

            updateHeightManager();
         }

         // if the footstep was adjusted, shift the CoM plan, if there is one.
         walkingMessageHandler.setPlanOffsetFromAdjustment(balanceManager.getEffectiveICPAdjustment());
      }

      if (requestSwingSpeedUp)
      {
         double swingTimeRemaining = requestSwingSpeedUpIfNeeded();
         balanceManager.updateSwingTimeRemaining(swingTimeRemaining);
      }
      boolean feetAreWellPositioned = legConfigurationManager.areFeetWellPositionedForCollapse(swingSide.getOppositeSide(),
                                                                                               nextFootstep.getSoleReferenceFrame());

      if (timeInState > fractionOfSwingToStraightenSwingLeg.getDoubleValue() * swingTime)
      {
         legConfigurationManager.straightenLegDuringSwing(swingSide);
      }
      if (timeInState > fractionOfSwingToCollapseStanceLeg.getDoubleValue() * swingTime && !legConfigurationManager.isLegCollapsed(supportSide)
            && feetAreWellPositioned)
      {
         legConfigurationManager.collapseLegDuringSwing(swingSide.getOppositeSide());
      }

      walkingMessageHandler.clearFootTrajectory();

      switchToToeOffIfPossible(supportSide);
   }

   @Override
   public boolean isDone(double timeInState)
   {
      hasMinimumTimePassed.set(hasMinimumTimePassed(timeInState));

      if (hasMinimumTimePassed.getBooleanValue() && footSwitches.get(swingSide).hasFootHitGround())
         return true;

      return finishSingleSupportWhenICPPlannerIsDone.getBooleanValue() && balanceManager.isICPPlanDone();
   }

   @Override
   public void onEntry()
   {
      balanceManager.clearICPPlan();
      footSwitches.get(swingSide).reset();
      balanceManager.setHoldSplitFractions(false);

      comHeightManager.setSupportLeg(swingSide.getOppositeSide());

      double defaultSwingTime = walkingMessageHandler.getDefaultSwingTime();
      double defaultTransferTime = walkingMessageHandler.getDefaultTransferTime();
      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();

      if (balanceManager.isRecoveringFromDoubleSupportFall())
      {
         swingTime = defaultSwingTime;
         footstepTiming.setTimings(swingTime, defaultTransferTime);
         balanceManager.packFootstepForRecoveringFromDisturbance(swingSide, defaultSwingTime, nextFootstep);
         nextFootstep.setTrajectoryType(TrajectoryType.DEFAULT);
         nextFootstep.setIsAdjustable(true);
         walkingMessageHandler.reportWalkingAbortRequested();
         walkingMessageHandler.clearFootsteps();
      }
      else
      {
         swingTime = walkingMessageHandler.getNextSwingTime();
         walkingMessageHandler.poll(nextFootstep, footstepTiming);
      }

      /** 1/08/2018 RJG this has to be done before calling #updateFootstepParameters() to make sure the contact points are up to date */
      feetManager.setContactStateForSwing(swingSide);

      updateFootstepParameters();

      balanceManager.minimizeAngularMomentumRateZ(minimizeAngularMomentumRateZDuringSwing.getValue());
      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.addFootstepToPlan(nextFootstep, footstepTiming);

      int stepsToAdd = Math.min(additionalFootstepsToConsider, walkingMessageHandler.getCurrentNumberOfFootsteps());
      boolean isLastStep = stepsToAdd == 0;
      for (int i = 0; i < stepsToAdd; i++)
      {
         walkingMessageHandler.peekFootstep(i, footsteps[i]);
         walkingMessageHandler.peekTiming(i, footstepTimings[i]);
         balanceManager.addFootstepToPlan(footsteps[i], footstepTimings[i]);
      }

      balanceManager.setICPPlanSupportSide(supportSide);
      balanceManager.initializeICPPlanForSingleSupport();

      updateHeightManager();


      feetManager.requestSwing(swingSide, nextFootstep, swingTime, balanceManager.getFinalDesiredCoMVelocity(), balanceManager.getFinalDesiredCoMAcceleration());

      if (feetManager.adjustHeightIfNeeded(nextFootstep))
      {
         walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
         feetManager.adjustSwingTrajectory(swingSide, nextFootstep, swingTime);
      }

      balanceManager.setSwingFootTrajectory(swingSide, feetManager.getSwingTrajectory(swingSide));
      if (balanceManager.isRecoveringFromDoubleSupportFall())
      {
         balanceManager.computeICPPlan();
         balanceManager.requestICPPlannerToHoldCurrentCoMInNextDoubleSupport();
      }


      legConfigurationManager.startSwing(swingSide);
      legConfigurationManager.useHighWeight(swingSide.getOppositeSide());
      legConfigurationManager.setStepDuration(supportSide, footstepTiming.getStepTime());

      if (isLastStep)
      {
         pelvisOrientationManager.initializeSwing(supportSide, swingTime, finalTransferTime, 0.0);
      }
      else
      {
         FootstepTiming nextTiming = footstepTimings[0];
         pelvisOrientationManager.initializeSwing(supportSide, swingTime, nextTiming.getTransferTime(), nextTiming.getSwingTime());
      }

      nextFootstep.getPose(desiredFootPoseInWorld);
      desiredFootPoseInWorld.changeFrame(worldFrame);

      actualFootPoseInWorld.setToZero(fullRobotModel.getSoleFrame(swingSide));
      actualFootPoseInWorld.changeFrame(worldFrame);
      walkingMessageHandler.reportFootstepStarted(swingSide, desiredFootPoseInWorld, actualFootPoseInWorld, swingTime);
   }

   @Override
   public void onExit()
   {
      balanceManager.resetPushRecovery();

      balanceManager.minimizeAngularMomentumRateZ(false);

      actualFootPoseInWorld.setToZero(fullRobotModel.getSoleFrame(swingSide));
      actualFootPoseInWorld.changeFrame(worldFrame);

      touchdownErrorCompensator.registerDesiredFootstepPosition(swingSide, desiredFootPoseInWorld.getPosition());

      walkingMessageHandler.reportFootstepCompleted(swingSide, desiredFootPoseInWorld, actualFootPoseInWorld, swingTime);
      walkingMessageHandler.registerCompletedDesiredFootstep(nextFootstep);

      MovingReferenceFrame soleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(nextFootstep.getRobotSide());
      tempOrientation.setIncludingFrame(nextFootstep.getFootstepPose().getOrientation());
      tempOrientation.changeFrame(soleZUpFrame);
      double pitch = tempOrientation.getPitch();

      if (doManualTouchdown())
      {
         // Get the initial condition from the swing trajectory
         FrameSE3TrajectoryPoint lastWaypoint = nextFootstep.getSwingTrajectory().get(nextFootstep.getSwingTrajectory().size() - 1);
         tempOrientation.setIncludingFrame(lastWaypoint.getOrientation());
         tempOrientation.changeFrame(soleZUpFrame);
         tempAngularVelocity.setIncludingFrame(lastWaypoint.getAngularVelocity());
         tempAngularVelocity.changeFrame(soleZUpFrame); // The y component is equivalent to the pitch rate since the yaw and roll rate are 0.0
      }
      else
      {
         // Get the initial condition from the robot state
         MovingReferenceFrame soleFrame = controllerToolbox.getReferenceFrames().getSoleFrame(nextFootstep.getRobotSide());
         tempOrientation.setToZero(soleFrame);
         tempOrientation.changeFrame(soleZUpFrame);
         tempAngularVelocity.setIncludingFrame(soleFrame.getTwistOfFrame().getAngularPart());
         tempAngularVelocity.changeFrame(soleZUpFrame);
      }
      double initialPitch = tempOrientation.getPitch();
      double initialPitchVelocity = tempAngularVelocity.getY();
      feetManager.touchDown(nextFootstep.getRobotSide(), initialPitch, initialPitchVelocity, pitch, footstepTiming.getTouchdownDuration());

      setYoVariablesToNaN();
   }

   private boolean doManualTouchdown()
   {
      return nextFootstep.getTrajectoryType() == TrajectoryType.WAYPOINTS && Precision.equals(nextFootstep.getSwingTrajectory().get(0).getTime(), 0.0);
   }

   private final FramePoint2D filteredDesiredCoP = new FramePoint2D(worldFrame);
   private final FramePoint2D desiredCoP = new FramePoint2D(worldFrame);
   private final FramePoint2D currentICP = new FramePoint2D(worldFrame);

   public void switchToToeOffIfPossible(RobotSide supportSide)
   {
      boolean shouldComputeToeLineContact = feetManager.shouldComputeToeLineContact();
      boolean shouldComputeToePointContact = feetManager.shouldComputeToePointContact();

      if (shouldComputeToeLineContact || shouldComputeToePointContact)
      {
         currentICP.setIncludingFrame(balanceManager.getCapturePoint());

         controllerToolbox.getDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(supportSide), desiredCoP);
         controllerToolbox.getFilteredDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(supportSide), filteredDesiredCoP);

         FramePoint3DReadOnly supportFootExitCMP = balanceManager.getFirstExitCMPForToeOff(false);

         feetManager.updateToeOffStatusSingleSupport(nextFootstep,
                                                     supportFootExitCMP,
                                                     balanceManager.getDesiredCMP(),
                                                     desiredCoP,
                                                     balanceManager.getDesiredICP(),
                                                     currentICP,
                                                     balanceManager.getFinalDesiredICP());

         if (feetManager.okForPointToeOff() && shouldComputeToePointContact)
            feetManager.requestPointToeOff(supportSide, supportFootExitCMP, filteredDesiredCoP);
         else if (feetManager.okForLineToeOff() && shouldComputeToeLineContact)
            feetManager.requestLineToeOff(supportSide, supportFootExitCMP, filteredDesiredCoP);

//         updateHeightManager();
      }
   }

   /**
    * Request the swing trajectory to speed up using
    * {@link us.ihmc.commonWalkingControlModules.capturePoint.ICPPlannerInterface#estimateTimeRemainingForStateUnderDisturbance(FramePoint2DReadOnly)}.
    * It is clamped w.r.t. to
    *
    * @return the current swing time remaining for the swing foot trajectory
    */
   private double requestSwingSpeedUpIfNeeded()
   {
      remainingSwingTimeAccordingToPlan.set(balanceManager.getTimeRemainingInCurrentState());

      double remainingTime = balanceManager.estimateTimeRemainingForSwingUnderDisturbance();
      estimatedRemainingSwingTimeUnderDisturbance.set(remainingTime);

      if (remainingTime > 1.0e-3)
      {
         double swingSpeedUpFactor = remainingSwingTimeAccordingToPlan.getDoubleValue() / remainingTime;
         return feetManager.requestSwingSpeedUp(swingSide, swingSpeedUpFactor);
      }
      else if (remainingSwingTimeAccordingToPlan.getDoubleValue() > 1.0e-3)
      {
         return feetManager.requestSwingSpeedUp(swingSide, Double.POSITIVE_INFINITY);
      }
      return remainingSwingTimeAccordingToPlan.getDoubleValue();
   }

   private void updateFootstepParameters()
   {
      // Update the contact states based on the footstep. If the footstep doesn't have any predicted contact points, then use the default ones in the ContactablePlaneBodies.
      controllerToolbox.updateContactPointsForUpcomingFootstep(nextFootstep);
      controllerToolbox.updateBipedSupportPolygons();

      pelvisOrientationManager.setTrajectoryTime(swingTime);
      pelvisOrientationManager.setUpcomingFootstep(nextFootstep);
      pelvisOrientationManager.updateTrajectoryFromFootstep(); // fixme this shouldn't be called when the footstep is updated

   }

   private void updateHeightManager()
   {
      TransferToAndNextFootstepsData transferToAndNextFootstepsData = walkingMessageHandler.createTransferToAndNextFootstepDataForSingleSupport(nextFootstep,
                                                                                                                                                swingSide);
      transferToAndNextFootstepsData.setComAtEndOfState(balanceManager.getFinalDesiredCoMPosition());
      double extraToeOffHeight = 0.0;
      if (feetManager.canDoSingleSupportToeOff(nextFootstep.getFootstepPose().getPosition(), swingSide) && feetManager.getToeOffManager().isSteppingUp())
         extraToeOffHeight = feetManager.getToeOffManager().getExtraCoMMaxHeightWithToes();
      comHeightManager.initialize(transferToAndNextFootstepsData, extraToeOffHeight);
   }

   private boolean hasMinimumTimePassed(double timeInState)
   {
      double minimumSwingTime;
      if (balanceManager.isRecoveringFromDoubleSupportFall())
         minimumSwingTime = 0.15;
      else
         minimumSwingTime = swingTime * minimumSwingFraction.getDoubleValue();

      return timeInState > minimumSwingTime;
   }
}