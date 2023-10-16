package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.BodyPartPoseStatusMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

/**
 * The IKRootCalculator class is responsible for computing and setting the root reference frame
 * for an inverse kinematics solver based on input pose status messages for the chest and pelvis.
 */
public class IKRootCalculator
{
   private final IHMCROS2Input<BodyPartPoseStatusMessage> chestPoseStatusSubscription;
   private final IHMCROS2Input<BodyPartPoseStatusMessage> pelvisPoseStatusSubscription;
   private final ROS2SyncedRobotModel syncedRobot;
   private BodyPartPoseStatusMessage chestPoseStatusMessage;
   private BodyPartPoseStatusMessage pelvisPoseStatusMessage;
   private MutableReferenceFrame latestChestFrame;
   private MutableReferenceFrame concurrentChestFrame;
   private MutableReferenceFrame latestCombinedPelvisAndChestFrame;
   private ReferenceFrame concurrentCombinedPelvisAndChestFrame;
   private ReferenceFrame rootReferenceFrame;

   public IKRootCalculator(ROS2PublishSubscribeAPI ros2, ROS2SyncedRobotModel syncedRobot)
   {
      this.syncedRobot = syncedRobot;

      chestPoseStatusSubscription = ros2.subscribe(BehaviorActionSequence.CHEST_POSE_STATUS);
      pelvisPoseStatusSubscription = ros2.subscribe(BehaviorActionSequence.PELVIS_POSE_VARIATION_STATUS);
   }

   /**
    * Retrieves the latest kinematics information from the subscribed pose status messages.
    * This method should be called before computing and setting the root reference frame.
    */
   public void getKinematicsInfo()
   {
      if (chestPoseStatusSubscription.getMessageNotification().poll())
         chestPoseStatusMessage = chestPoseStatusSubscription.getLatest();
      else
         chestPoseStatusMessage = null;

      if (pelvisPoseStatusSubscription.getMessageNotification().poll())
         pelvisPoseStatusMessage = pelvisPoseStatusSubscription.getLatest();
      else
         pelvisPoseStatusMessage = null;
   }

   /**
    * Computes and sets the root reference frame based on chest and pelvis pose status messages.
    */
   public void computeRoot()
   {
      if (chestPoseStatusMessage != null)
      {
         boolean isChestCurrentAndConcurrent = chestPoseStatusMessage.getCurrentAndConcurrent();
         latestChestFrame = new MutableReferenceFrame(syncedRobot.getReferenceFrames().getChestFrame().getParent());
         latestChestFrame.update(transformToParent -> MessageTools.toEuclid(chestPoseStatusMessage.getTransformToParent(), transformToParent));
         if (isChestCurrentAndConcurrent)
            concurrentChestFrame = latestChestFrame;
         else
            concurrentChestFrame = null;
      }

      if (pelvisPoseStatusMessage != null)
      {
         boolean isPelvisCurrentAndConcurrent = pelvisPoseStatusMessage.getCurrentAndConcurrent();
         FramePose3D pelvisFramePoseVariation = new FramePose3D(syncedRobot.getReferenceFrames().getPelvisFrame().getParent(),
                                                                MessageTools.toEuclid(pelvisPoseStatusMessage.getTransformToParent()));
         if (concurrentChestFrame != null)
         {
            latestCombinedPelvisAndChestFrame = new MutableReferenceFrame(concurrentChestFrame.getReferenceFrame().getParent());
            latestCombinedPelvisAndChestFrame.update(transformToParent -> copyTransform(concurrentChestFrame.getTransformToParent(), transformToParent));
         }
         else
            latestCombinedPelvisAndChestFrame = new MutableReferenceFrame(syncedRobot.getFullRobotModel().getChest()
                                                                                     .getParentJoint()
                                                                                     .getFrameAfterJoint());
         latestCombinedPelvisAndChestFrame.update(transformToParent -> updateIKChestTransform(latestCombinedPelvisAndChestFrame,
                                                                                              pelvisFramePoseVariation,
                                                                                              transformToParent));
         if (isPelvisCurrentAndConcurrent)
            concurrentCombinedPelvisAndChestFrame = latestCombinedPelvisAndChestFrame.getReferenceFrame();
         else
            concurrentCombinedPelvisAndChestFrame = null;
      }

      if (concurrentCombinedPelvisAndChestFrame == null)
         rootReferenceFrame = concurrentChestFrame == null ? syncedRobot.getFullRobotModel().getChest().getParentJoint().getFrameAfterJoint() : concurrentChestFrame.getReferenceFrame();
      else
         rootReferenceFrame = concurrentCombinedPelvisAndChestFrame;
   }

   public ReferenceFrame getRoot()
   {
      return rootReferenceFrame;
   }

   /**
    * Updates the IKChestTransform based on the pelvisFramePoseVariation and the current IKChestFrame.
    *
    * @param IKChestFrame           The reference frame of the IK chest.
    * @param pelvisFramePoseVariation The variation in the pelvis frame's pose.
    * @param IKChestTransform       The transform to update with the new chest pose.
    */
   private void updateIKChestTransform(MutableReferenceFrame IKChestFrame, FramePose3D pelvisFramePoseVariation, RigidBodyTransform IKChestTransform)
   {
      if (pelvisFramePoseVariation.getReferenceFrame() != IKChestFrame.getReferenceFrame().getParent())
      {
         pelvisFramePoseVariation.changeFrame(IKChestFrame.getReferenceFrame().getParent());
      }
      IKChestTransform.set(IKChestFrame.getReferenceFrame().getTransformToRoot());
      IKChestTransform.getTranslation().setZ(IKChestTransform.getTranslationZ() + pelvisFramePoseVariation.getTranslationZ());
   }

   private void copyTransform(RigidBodyTransform transformToCopy, RigidBodyTransform otherTransform)
   {
      otherTransform.set(transformToCopy);
   }
}
