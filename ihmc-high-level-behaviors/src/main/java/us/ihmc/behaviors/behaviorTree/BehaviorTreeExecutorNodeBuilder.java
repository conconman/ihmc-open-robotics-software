package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.behaviors.tools.ROS2HandWrenchCalculator;
import us.ihmc.behaviors.tools.walkingController.WalkingFootstepTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.SideDependentList;

public class BehaviorTreeExecutorNodeBuilder implements BehaviorTreeNodeStateBuilder
{
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final WalkingFootstepTracker footstepTracker;
   private final SideDependentList<ROS2HandWrenchCalculator> handWrenchCalculators;
   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final ROS2ControllerHelper ros2ControllerHelper;

   public BehaviorTreeExecutorNodeBuilder(DRCRobotModel robotModel,
                                          ROS2SyncedRobotModel syncedRobot,
                                          ReferenceFrameLibrary referenceFrameLibrary,
                                          WalkingFootstepTracker footstepTracker,
                                          SideDependentList<ROS2HandWrenchCalculator> handWrenchCalculators,
                                          FootstepPlanningModule footstepPlanner,
                                          FootstepPlannerParametersBasics footstepPlannerParameters,
                                          WalkingControllerParameters walkingControllerParameters,
                                          ROS2ControllerHelper ros2ControllerHelper)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.referenceFrameLibrary = referenceFrameLibrary;
      this.footstepTracker = footstepTracker;
      this.handWrenchCalculators = handWrenchCalculators;
      this.footstepPlanner = footstepPlanner;
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.walkingControllerParameters = walkingControllerParameters;
      this.ros2ControllerHelper = ros2ControllerHelper;
   }

   @Override
   public BehaviorTreeNodeExecutor createNode(Class<?> nodeType, long id)
   {
      BehaviorActionSequence sequence = null; // TODO ???? Probably need the parent sequence node?

      if (nodeType == ArmJointAnglesActionDefinition.class)
      {
         return new ArmJointAnglesActionExecutor(id, sequence, robotModel, ros2ControllerHelper);
      }
      if (nodeType == ChestOrientationActionDefinition.class)
      {
         return new ChestOrientationActionExecutor(id, sequence, ros2ControllerHelper, syncedRobot, referenceFrameLibrary);
      }
      if (nodeType == FootstepPlanActionDefinition.class)
      {
         return new FootstepPlanActionExecutor(id,
                                               sequence,
                                               ros2ControllerHelper,
                                               syncedRobot,
                                               footstepTracker,
                                               referenceFrameLibrary,
                                               walkingControllerParameters);
      }
      if (nodeType == HandPoseActionDefinition.class)
      {
         return new HandPoseActionExecutor(id, sequence, ros2ControllerHelper, referenceFrameLibrary, robotModel, syncedRobot, handWrenchCalculators);
      }
      if (nodeType == HandWrenchActionDefinition.class)
      {
         return new HandWrenchActionExecutor(id, sequence, ros2ControllerHelper);
      }
      if (nodeType == PelvisHeightPitchActionDefinition.class)
      {
         return new PelvisHeightPitchActionExecutor(id, sequence, ros2ControllerHelper, referenceFrameLibrary, syncedRobot);
      }
      if (nodeType == SakeHandCommandActionDefinition.class)
      {
         return new SakeHandCommandActionExecutor(id, sequence, ros2ControllerHelper);
      }
      if (nodeType == WaitDurationActionDefinition.class)
      {
         return new WaitDurationActionExecutor(id, sequence);
      }
      if (nodeType == WalkActionDefinition.class)
      {
         return new WalkActionExecutor(id,
                                       sequence,
                                       ros2ControllerHelper,
                                       syncedRobot,
                                       footstepTracker,
                                       footstepPlanner,
                                       footstepPlannerParameters,
                                       walkingControllerParameters,
                                       referenceFrameLibrary);
      }

      return null;
   }
}