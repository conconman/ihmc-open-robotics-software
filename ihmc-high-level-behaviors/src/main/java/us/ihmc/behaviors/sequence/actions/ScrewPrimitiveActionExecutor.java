package us.ihmc.behaviors.sequence.actions;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionCalculator;
import us.ihmc.behaviors.tools.ROS2HandWrenchCalculator;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ScrewPrimitiveActionExecutor extends ActionNodeExecutor<ScrewPrimitiveActionState, ScrewPrimitiveActionDefinition>
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final FramePose3D desiredHandControlPose = new FramePose3D();
   private final FramePose3D syncedHandControlPose = new FramePose3D();
   private final SideDependentList<ROS2HandWrenchCalculator> handWrenchCalculators;
   private double startPositionDistanceToGoal;
   private double startOrientationDistanceToGoal;
   private final BehaviorActionCompletionCalculator completionCalculator = new BehaviorActionCompletionCalculator();

   public ScrewPrimitiveActionExecutor(long id,
                                       CRDTInfo crdtInfo,
                                       WorkspaceResourceDirectory saveFileDirectory,
                                       ROS2ControllerHelper ros2ControllerHelper,
                                       ReferenceFrameLibrary referenceFrameLibrary,
                                       DRCRobotModel robotModel,
                                       ROS2SyncedRobotModel syncedRobot,
                                       SideDependentList<ROS2HandWrenchCalculator> handWrenchCalculators)
   {
      super(new ScrewPrimitiveActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.handWrenchCalculators = handWrenchCalculators;
   }

   @Override
   public void update()
   {
      super.update();

      getState().setCanExecute(getState().getScrewFrame().isChildOfWorld());
   }

   @Override
   public void triggerActionExecution()
   {
      if (getState().getScrewFrame().isChildOfWorld())
      {
//         desiredHandControlPose.setFromReferenceFrame(getState().getPalmFrame().getReferenceFrame());
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getDefinition().getSide()));
         startPositionDistanceToGoal = syncedHandControlPose.getTranslation().differenceNorm(desiredHandControlPose.getTranslation());
         startOrientationDistanceToGoal = syncedHandControlPose.getRotation().distance(desiredHandControlPose.getRotation(), true);
      }
      else
      {
         LogTools.error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      if (getState().getScrewFrame().isChildOfWorld())
      {
//         desiredHandControlPose.setFromReferenceFrame(getState().getPalmFrame().getReferenceFrame());
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getDefinition().getSide()));


         // TODO: Completion criteria
//         getState().setIsExecuting();


         getState().setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
         getState().setStartPositionDistanceToGoal(startPositionDistanceToGoal);
         getState().setCurrentOrientationDistanceToGoal(completionCalculator.getRotationError());
         getState().setCurrentPositionDistanceToGoal(completionCalculator.getTranslationError());
         getState().setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
         getState().setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
      }
   }
}
