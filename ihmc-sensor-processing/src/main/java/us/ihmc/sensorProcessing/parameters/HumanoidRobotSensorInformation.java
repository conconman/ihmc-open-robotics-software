package us.ihmc.sensorProcessing.parameters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public interface HumanoidRobotSensorInformation extends AvatarRobotRosVisionSensorInformation, HumanoidForceSensorInformation, IMUSensorInformation
{
   public default RigidBodyTransform getSteppingCameraTransform()
   {
      return new RigidBodyTransform();
   }

   public default ReferenceFrame getSteppingCameraParentFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return referenceFrames.getChestFrame();
   }

   public default ReferenceFrame getSteppingCameraFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return ReferenceFrameTools.constructFrameWithChangingTransformToParent("steppingCamera",
                                                                             getSteppingCameraParentFrame(referenceFrames),
                                                                             getSteppingCameraTransform());
   }

   public default RigidBodyTransform getObjectDetectionCameraTransform()
   {
      return new RigidBodyTransform();
   }

   public default ReferenceFrame getObjectDetectionCameraParentFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return referenceFrames.getChestFrame();
   }

   public default ReferenceFrame getObjectDetectionCameraFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return ReferenceFrameTools.constructFrameWithChangingTransformToParent("objectDetectionCamera",
                                                                             getObjectDetectionCameraParentFrame(referenceFrames),
                                                                             getObjectDetectionCameraTransform());
   }

   public default RigidBodyTransform getExperimentalCameraTransform()
   {
      return new RigidBodyTransform();
   }

   public default ReferenceFrame getExperimentalCameraParentFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return referenceFrames.getChestFrame();
   }

   public default ReferenceFrame getExperimentalCameraFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return ReferenceFrameTools.constructFrameWithChangingTransformToParent("experimentalCamera",
                                                                             getExperimentalCameraParentFrame(referenceFrames),
                                                                             getExperimentalCameraTransform());
   }

   public default RigidBodyTransform getOusterLidarTransform()
   {
      return new RigidBodyTransform();
   }

   public default ReferenceFrame getOusterLidarParentFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return referenceFrames.getChestFrame();
   }

   public default ReferenceFrame getOusterLidarFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return ReferenceFrameTools.constructFrameWithChangingTransformToParent("ousterLidar",
                                                                             getOusterLidarParentFrame(referenceFrames),
                                                                             getOusterLidarTransform());
   }

   public default String getHeadCameraName()
   {
      return null;
   }
}
