package us.ihmc.behaviors.targetFollowing;

import geometry_msgs.PoseStamped;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.BehaviorDefinition;
import us.ihmc.behaviors.BehaviorInterface;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.tools.behaviorTree.ResettingNode;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.tools.Timer;
import us.ihmc.utilities.ros.RosTools;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.targetFollowing.TargetFollowingBehaviorAPI.*;

public class TargetFollowingBehavior extends ResettingNode implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Target Following",
                                                                              TargetFollowingBehavior::new,
                                                                              TargetFollowingBehaviorAPI.API);
   private final BehaviorHelper helper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final LookAndStepBehavior lookAndStepBehavior;
   private final TargetFollowingBehaviorParameters targetFollowingParameters;
   private final AtomicReference<PoseStamped> latestSemanticTargetPoseReference = new AtomicReference<>();
   private final Timer lookAndStepGoalSubmissionTimer = new Timer();
   private final FramePose3D targetPoseGroundProjection = new FramePose3D();
   private final FramePose3D approachPose = new FramePose3D();
   private final FramePose3D robotMidFeetUnderPelvisPose = new FramePose3D();

   public TargetFollowingBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      syncedRobot = helper.newSyncedRobot();
      LogTools.info("Constructing");
      targetFollowingParameters = new TargetFollowingBehaviorParameters();
      lookAndStepBehavior = new LookAndStepBehavior(helper);
      addChild(lookAndStepBehavior);
      helper.subscribeViaCallback(TargetFollowingParameters, parameters ->
      {
         helper.getOrCreateStatusLogger().info("Accepting new target following parameters");
         this.targetFollowingParameters.setAllFromStrings(parameters);
      });
      helper.getROS1Helper().subscribeToPoseViaCallback(RosTools.SEMANTIC_TARGET_POSE, latestSemanticTargetPoseReference::set);
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      PoseStamped latestSemanticTargetPose = latestSemanticTargetPoseReference.getAndSet(null);
      if (latestSemanticTargetPose != null)
      {
         syncedRobot.update();
         robotMidFeetUnderPelvisPose.setToZero(syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame());
         robotMidFeetUnderPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

         targetPoseGroundProjection.setToZero(syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame());
         RosTools.toEuclid(latestSemanticTargetPose.getPose(), targetPoseGroundProjection);
         targetPoseGroundProjection.changeFrame(ReferenceFrame.getWorldFrame());
         targetPoseGroundProjection.getPosition().setZ(robotMidFeetUnderPelvisPose.getZ());

         approachPose.set(targetPoseGroundProjection);
         Vector3D fromTarget = new Vector3D();
         fromTarget.sub(robotMidFeetUnderPelvisPose.getPosition(), targetPoseGroundProjection.getPosition());
         fromTarget.normalize();
         fromTarget.scale(targetFollowingParameters.getMinimumDistanceToKeepFromTarget());
         approachPose.getPosition().add(fromTarget);
         helper.publish(TargetApproachPose, new Pose3D(approachPose));

         if (!lookAndStepGoalSubmissionTimer.isRunning(targetFollowingParameters.getLookAndStepGoalUpdatePeriod()))
         {
            lookAndStepBehavior.acceptGoal(approachPose);
            lookAndStepGoalSubmissionTimer.reset();
         }
      }

      return lookAndStepBehavior.tick();
   }

   @Override
   public void reset()
   {

   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }

   @Override
   public void destroy()
   {
      lookAndStepBehavior.destroy();
   }
}


