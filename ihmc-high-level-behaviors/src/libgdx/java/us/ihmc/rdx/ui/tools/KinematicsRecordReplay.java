package us.ihmc.rdx.ui.tools;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import org.lwjgl.openvr.InputDigitalActionData;
import perception_msgs.msg.dds.DetectableSceneNodesMessage;
import us.ihmc.behaviors.tools.TrajectoryRecordReplay;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraphAPI;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class KinematicsRecordReplay
{
   private final TrajectoryRecordReplay trajectoryRecorder = new TrajectoryRecordReplay("", 1);
   private final ImString recordPath = new ImString(Paths.get(System.getProperty("user.home"), ".ihmc/logs").toString());
   private final ImBoolean enablerRecording = new ImBoolean(false);
   private boolean isRecording = false;
   private final ImString replayPath = new ImString(Paths.get(System.getProperty("user.home"), ".ihmc/logs/1.csv").toString());
   private final ImBoolean enablerReplay = new ImBoolean(false);
   private boolean isReplaying = false;
   private final ImBoolean enabledKinematicsStreaming;
   private boolean isUserMoving = false;
   private final List<List<Pose3DReadOnly>> framesToRecordHistory = new ArrayList<>();
   private int partId = 0; // identifier of current frame, used to now what body part among numberOfParts we are currently handling
   private final ROS2PublishSubscribeAPI ros2;
   private final IHMCROS2Input<DetectableSceneNodesMessage> detectableSceneObjectsSubscription;
   private boolean sceneNodeLocked = false;
   private ReferenceFrame sceneNodeFrame;
   private final HashMap<String, FramePose3D> previousFramePose = new HashMap<>();


   public KinematicsRecordReplay(ROS2PublishSubscribeAPI ros2, ImBoolean enabledKinematicsStreaming, int numberOfParts)
   {
      this.ros2 = ros2;
      this.enabledKinematicsStreaming = enabledKinematicsStreaming;
      trajectoryRecorder.setNumberOfParts(numberOfParts);
      for (int n = 0; n < numberOfParts; n++)
         framesToRecordHistory.add(new ArrayList<>());

      detectableSceneObjectsSubscription = ros2.subscribe(SceneGraphAPI.DETECTABLE_SCENE_NODES);
      trajectoryRecorder.setDoneReplay(true);
   }

   public void processRecordReplayInput(InputDigitalActionData triggerButton)
   {
      // check streaming is on, recording is on and trigger button has been pressed once. if button is pressed again recording is stopped
      if (enabledKinematicsStreaming.get() && enablerRecording.get() && triggerButton.bChanged() && !triggerButton.bState())
      {
         isRecording = !isRecording;
         checkIfSceneNodeIsAvailable();

         // check if recording file path has been set to a different one from previous recording. In case update file path.
         if (trajectoryRecorder.hasSavedRecording() && !(trajectoryRecorder.getPath().equals(recordPath.get())))
            trajectoryRecorder.setPath(recordPath.get()); //recorder is reset when changing path
      }
      // check replay is on and trigger button has been pressed once. if button is pressed again replay is stopped
      if (enablerReplay.get() && triggerButton.bChanged() && !triggerButton.bState())
      {
         isReplaying = !isReplaying;
         checkIfSceneNodeIsAvailable();

         // check if replay file has been set to a different one from previous replay. In case update file path.
         if (trajectoryRecorder.hasDoneReplay() && !(trajectoryRecorder.getPath().equals(replayPath.get())))
            trajectoryRecorder.setPath(replayPath.get()); // replayer is reset when changing path
      }
   }

   private void checkIfSceneNodeIsAvailable()
   {
      if (detectableSceneObjectsSubscription.getMessageNotification().poll() && !sceneNodeLocked)
      {
         DetectableSceneNodesMessage detectableSceneNodeMessage = detectableSceneObjectsSubscription.getMessageNotification().read();
         for (var sceneNodeMessage : detectableSceneNodeMessage.getDetectableSceneNodes())
         {
            // TODO. update this once Panel and LeverHandle are unified into single detectable "Door"
            if (sceneNodeMessage.currently_detected_ && !sceneNodeMessage.name_.toString().contains("Panel") && !sceneNodeMessage.name_.toString().contains("Frame"))
            {
               RigidBodyTransform objectTransformToWorld = new RigidBodyTransform();
               MessageTools.toEuclid(sceneNodeMessage.getTransformToWorld(), objectTransformToWorld);
               sceneNodeFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                         objectTransformToWorld);
               break;
            }
         }
         sceneNodeLocked = true;
      }
   }

   public void framePoseToRecord(FramePose3DReadOnly framePose, String frameName)
   {
      if (isRecording)
      {
         if (isMoving(framePose)) //check from framePose if the user is moving
         { // we want to start the recording as soon as the user starts moving, recordings with different initial pauses can lead to bad behaviors when used for learning
            framePose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
            FramePose3D frameToRecord = new FramePose3D(framePose);
            ensureOrientationContinuity(frameToRecord, frameName);
            // transform to object reference frame if using object detection
            if (sceneNodeFrame != null)
               frameToRecord.changeFrame(sceneNodeFrame);

            // Store trajectories in file: store a setpoint per timestep until trigger button is pressed again
            double[] dataTrajectories = new double[7];
            frameToRecord.getOrientation().get(dataTrajectories);
            frameToRecord.getPosition().get(4, dataTrajectories);
            trajectoryRecorder.record(dataTrajectories);
         }
      }
      else if (!(trajectoryRecorder.hasSavedRecording()))
      {
         trajectoryRecorder.concatenateData();
         trajectoryRecorder.saveRecording();
         isUserMoving = false;
         sceneNodeLocked = false;
         sceneNodeFrame = null;
      }
   }

   private void ensureOrientationContinuity(FramePose3D frameToCheck, String frameName)
   {
      if (previousFramePose.containsKey(frameName))
      {
         // Check that quaternion is not changing 2pi range. Even if q = -q, the observed motion has to be continuous
         if (Math.signum(previousFramePose.get(frameName).getOrientation().getS() * frameToCheck.getOrientation().getS()) == -1
             && Math.signum(previousFramePose.get(frameName).getOrientation().getZ() * frameToCheck.getOrientation().getZ()) == -1
             && Math.signum(previousFramePose.get(frameName).getOrientation().getY() * frameToCheck.getOrientation().getY()) == -1
             && Math.signum(previousFramePose.get(frameName).getOrientation().getX() * frameToCheck.getOrientation().getX()) == -1)
            frameToCheck.getOrientation().negate();

         previousFramePose.get(frameName).set(frameToCheck);
      }
      else
      {
         previousFramePose.put(frameName,new FramePose3D());
      }
   }

   private boolean isMoving(FramePose3DReadOnly framePose)
   {
      if (!isUserMoving)
      {
         Pose3D lastFramePose = new Pose3D(framePose);
         framesToRecordHistory.get(partId).add(lastFramePose);
         // check if last value of frame pose translated by 4cm with respect to first value of frame pose
         if (framesToRecordHistory.get(partId).size() > 1)
         {
            double distance = (framesToRecordHistory.get(partId).get(framesToRecordHistory.get(partId).size() - 1)).getTranslation()
                                                                                                                   .distance(framesToRecordHistory.get(partId)
                                                                                                                                                  .get(0)
                                                                                                                                                  .getTranslation());
            isUserMoving = distance > 0.04;
         }
         if (!isUserMoving) // if still not moving analyze next frame at next call
         {
            partId++;
            if (partId >= framesToRecordHistory.size())
               partId = 0;
         }
      }
      if (isUserMoving && partId > 0)  // we want to record the frames of all parts at each time step, if the user starts moving only after we parsed the frame of a previous part
      { // this clause will activate the recording at the next step, preventing start recording while skipping the frame of a part, which would make the trajectory recorder crash
         partId++;
         if (partId >= framesToRecordHistory.size())
            partId = 0;
         return false;
      }
      return isUserMoving && partId == 0;
   }

   public void framePoseToPack(FramePose3D framePose)
   {
      framePose.setFromReferenceFrame(ReferenceFrame.getWorldFrame());
      // Read file with stored trajectories: read set point per timestep until file is over
      double[] dataPoint = trajectoryRecorder.play(true); //play split data (a body part per time)
      if (sceneNodeFrame != null)
         framePose.changeFrame(sceneNodeFrame);
      // [0,1,2,3] quaternion of body segment; [4,5,6] position of body segment
      framePose.getOrientation().set(dataPoint);
      framePose.getPosition().set(4, dataPoint);
      if (sceneNodeFrame != null)
         framePose.changeFrame(ReferenceFrame.getWorldFrame());
      if (trajectoryRecorder.hasDoneReplay())
      {
         isReplaying = false;
         enablerReplay.set(false);
      }
   }

   public void renderRecordWidgets(ImGuiUniqueLabelMap labels)
   {
      if (ImGui.checkbox(labels.get("Record motion"), enablerRecording))
      {
         setRecording(enablerRecording.get());
      }
      ImGui.sameLine();
      ImGui.inputText(labels.get("Record folder"), recordPath);
   }

   public void renderReplayWidgets(ImGuiUniqueLabelMap labels)
   {
      if (ImGui.checkbox(labels.get("Replay motion"), enablerReplay))
      {
         setReplay(enablerReplay.get());
      }
      ImGui.sameLine();
      ImGui.inputText(labels.get("Replay file"), replayPath);
   }

   private void setRecording(boolean enablerRecording)
   {
      if (enablerRecording != this.enablerRecording.get())
         this.enablerRecording.set(enablerRecording);
      if (enablerRecording)
         this.enablerReplay.set(false); // check no concurrency replay and record
   }

   public void setReplay(boolean enablerReplay)
   {
      if (enablerReplay != this.enablerReplay.get())
         this.enablerReplay.set(enablerReplay);
      if (enablerReplay)
      {
         if (enablerRecording.get() || enabledKinematicsStreaming.get())
            this.enablerReplay.set(false); // check no concurrency replay and record/streaming
      }
   }

   public ImBoolean isRecordingEnabled()
   {
      return enablerRecording;
   }

   public ImBoolean isReplayingEnabled()
   {
      return enablerReplay;
   }

   public boolean isRecording()
   {
      return isRecording;
   }

   public boolean isReplaying()
   {
      return isReplaying;
   }
}
