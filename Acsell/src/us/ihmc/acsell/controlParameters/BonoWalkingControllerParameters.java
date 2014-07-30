package us.ihmc.acsell.controlParameters;

import javax.media.j3d.Transform3D;

import us.ihmc.acsell.parameters.BonoPhysicalProperties;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;

/**
 * Created by dstephen on 2/14/14.
 */
public class BonoWalkingControllerParameters implements WalkingControllerParameters
{

   private final SideDependentList<Transform3D> handPosesWithRespectToChestFrame = new SideDependentList<Transform3D>();

   private final boolean runningOnRealRobot;
   
   public BonoWalkingControllerParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      
      for(RobotSide robotSide : RobotSide.values())
      {
         handPosesWithRespectToChestFrame.put(robotSide, new Transform3D());
      }
   }

   @Override
   public SideDependentList<Transform3D> getDesiredHandPosesWithRespectToChestFrame()
   {
      return handPosesWithRespectToChestFrame;
   }

   @Override
   public boolean stayOnToes()
   {
      return false; // Not working for now
   }

   @Override
   public boolean doToeOffIfPossible()
   {
      return true;
   }

   @Override
   public double getMaximumToeOffAngle()
   {
      return Math.toRadians(45.0);
   }

   @Override
   public boolean doToeTouchdownIfPossible()
   {
      return false;
   }

   @Override
   public double getToeTouchdownAngle()
   {
      return Math.toRadians(20.0);
   }

   @Override
   public boolean doHeelTouchdownIfPossible()
   {
      return false;
   }

   @Override
   public double getHeelTouchdownAngle()
   {
      return Math.toRadians(-20.0);
   }

   @Override
   public String[] getDefaultHeadOrientationControlJointNames()
   {
      return new String[0];
   }

   @Override
   public String[] getDefaultChestOrientationControlJointNames()
   {
      return new String[0];
   }

   @Override
   public boolean checkOrbitalEnergyCondition()
   {
      return false;
   }

   private final double minimumHeightAboveGround = 0.595 + 0.03;
   private double nominalHeightAboveGround = 0.675 + 0.03;
   private final double maximumHeightAboveGround = 0.735 + 0.03;
   private final double additionalOffsetHeightBono = 0.05;

   @Override
   public double minimumHeightAboveAnkle()
   {
      return minimumHeightAboveGround + additionalOffsetHeightBono;
   }

   @Override
   public double nominalHeightAboveAnkle()
   {
      return nominalHeightAboveGround + additionalOffsetHeightBono;
   }

   @Override
   public double maximumHeightAboveAnkle()
   {
      return maximumHeightAboveGround + additionalOffsetHeightBono;
   }

   public void setNominalHeightAboveAnkle(double nominalHeightAboveAnkle)
   {
      this.nominalHeightAboveGround = nominalHeightAboveAnkle;
   }

   @Override
   public double getGroundReactionWrenchBreakFrequencyHertz()
   {
      return 7.0;
   }

   @Override
   public boolean resetDesiredICPToCurrentAtStartOfSwing()
   {
      return false;
   }

   @Override
   public double getUpperNeckPitchLimit()
   {
      return 0.0;
   }

   @Override
   public double getLowerNeckPitchLimit()
   {
      return 0.0;
   }

   @Override
   public double getHeadYawLimit()
   {
      return 0.0;
   }

   @Override
   public double getHeadRollLimit()
   {
      return 0.0;
   }

   @Override
   public String getJointNameForExtendedPitchRange()
   {
      return null;
   }

   @Override
   public boolean finishSwingWhenTrajectoryDone()
   {
      return false;
   }

   @Override
   public double getFootForwardOffset()
   {
      return BonoPhysicalProperties.footForward;
   }

   @Override
   public double getFootBackwardOffset()
   {
      return BonoPhysicalProperties.footBack;
   }

   @Override
   public double getAnkleHeight()
   {
      return BonoPhysicalProperties.ankleHeight;
   }

   @Override
   public double getLegLength()
   {
      return BonoPhysicalProperties.legLength;
   }

   @Override
   public double getMinLegLengthBeforeCollapsingSingleSupport()
   {
      //TODO: Useful values
      return 0.1;
   }

   @Override
   public double getFinalToeOffPitchAngularVelocity()
   {
      return 3.5;
   }

   @Override
   public double getInPlaceWidth()
   {
      return 0.25;
   }

   @Override
   public double getDesiredStepForward()
   {
      return 0.3; //0.5; //0.35;
   }

   @Override
   public double getMaxStepLength()
   {
      return 0.4; //0.6; //0.5; //0.35;
   }

   @Override
   public double getMinStepWidth()
   {
      return 0.15;
   }
   
   @Override
   public double getFootSwitchCoPThresholdFraction()
   {
	   return 0.02;
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.4; //0.5; //0.4;
   }

   @Override
   public double getStepPitch()
   {
      return 0.0;
   }

   @Override
   public double getCaptureKpParallelToMotion()
   {
      return 1.0;
   }

   @Override
   public double getCaptureKpOrthogonalToMotion()
   {
      return 1.0;
   }

   @Override
   public double getCaptureKi()
   {
      return 4.0;
   }

   @Override
   public double getCaptureKiBleedoff()
   {
      return 0.9;
   }

   @Override
   public double getCaptureFilterBreakFrequencyInHz()
   {
      return 16.0; //Double.POSITIVE_INFINITY;
   }

   @Override
   public double getCMPRateLimit()
   {
      return 60.0;
   }

   @Override
   public double getCMPAccelerationLimit()
   {
      return 2000.0;
   }

   @Override
   public double getKpCoMHeight()
   {
      return 50.0;
   }

   @Override
   public double getZetaCoMHeight()
   {
      return 1.0;
   }

   @Override
   public double getDefaultDesiredPelvisPitch()
   {
      return 0.0;
   }

   @Override
   public double getKpPelvisOrientation()
   {
      return 100.0;
   }

   @Override
   public double getZetaPelvisOrientation()
   {
      return 0.8; //1.0;
   }

   @Override
   public double getMaxAccelerationPelvisOrientation()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getMaxJerkPelvisOrientation()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getKpHeadOrientation()
   {
      return 40.0;
   }

   @Override
   public double getZetaHeadOrientation()
   {
      return 0.8; //1.0;
   }

   @Override
   public double getTrajectoryTimeHeadOrientation()
   {
      return 3.0;
   }

   @Override
   public double getKpUpperBody()
   {
      return 100.0;
   }

   @Override
   public double getZetaUpperBody()
   {
      return 0.8; //1.0;
   }

   @Override
   public double getMaxAccelerationUpperBody()
   {
      return Double.POSITIVE_INFINITY; //100.0;
   }

   @Override
   public double getMaxJerkUpperBody()
   {
      return Double.POSITIVE_INFINITY;//270.0; //1000.0;
   }

   @Override
   public double getSwingKpXY()
   {
      return 100.0;
   }

   @Override
   public double getSwingKpZ()
   {
      return 200.0;
   }

   @Override
   public double getSwingKpOrientation()
   {
      return 200.0;
   }

   @Override
   public double getSwingZetaXYZ()
   {
      return 0.7;
   }

   @Override
   public double getSwingZetaOrientation()
   {
      return 0.7;
   }

   @Override
   public double getHoldKpXY()
   {
      return 100.0;
   }

   @Override
   public double getHoldKpOrientation()
   {
      return 100.0;
   }

   @Override
   public double getHoldZeta()
   {
      return 1.0;
   }

   @Override
   public double getSwingMaxPositionAcceleration()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getSwingMaxPositionJerk()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getSwingMaxOrientationAcceleration()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getSwingMaxOrientationJerk()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getSupportSingularityEscapeMultiplier()
   {
      return 30; 
   }

   @Override
   public double getSwingSingularityEscapeMultiplier()
   {
      return runningOnRealRobot ? 50.0 : 200.0;
   }

   @Override
   public boolean doPrepareManipulationForLocomotion()
   {
      return true;
   }

   @Override
   public double getToeOffKpXY()
   {
      return 100.0;
   }

   @Override
   public double getToeOffKpOrientation()
   {
      return 200.0;
   }

   @Override
   public double getToeOffZeta()
   {
      return 0.4;
   }

   @Override
   public boolean isRunningOnRealRobot()
   {
      return runningOnRealRobot;
   }

   @Override
   public double getDefaultTransferTime()
   {
      return 0.25; // 1.5; //
   }

   @Override
   public double getDefaultSwingTime()
   {
      return 0.6; // 1.5; //
   }

   @Override
   public double getPelvisPitchUpperLimit()
   {
      return 0;
   }

   @Override
   public double getPelvisPitchLowerLimit()
   {
      return 0;
   }

   @Override
   public boolean isPelvisPitchReversed()
   {
      return false;
   }

   @Override
   public double getFootWidth()
   {
      return BonoPhysicalProperties.footWidth;
   }

   @Override
   public double getToeWidth()
   {
      return BonoPhysicalProperties.footWidth;
   }

   @Override
   public double getFootLength()
   {
      return BonoPhysicalProperties.footForward + BonoPhysicalProperties.footBack;
   }

   @Override
   public double getFoot_start_toetaper_from_back()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public double getSideLengthOfBoundingBoxForFootstepHeight()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public double getSwingHeightMaxForPushRecoveryTrajectory()
   {
	   return 0.15;
   }

   @Override
   public double getDesiredTouchdownVelocity()
   {
      return -0.3;
   }

   @Override
   public double getContactThresholdForce()
   {
      return 5.0;
   }
}
