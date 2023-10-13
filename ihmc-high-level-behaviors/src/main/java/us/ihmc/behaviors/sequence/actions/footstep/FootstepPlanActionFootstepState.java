package us.ihmc.behaviors.sequence.actions.footstep;

import behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;

public class FootstepPlanActionFootstepState
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final FootstepPlanActionState footstepPlan;
   private final FootstepPlanActionFootstepDefinition definition;
   private final DetachableReferenceFrame soleFrame;
   private int index = -1;

   public FootstepPlanActionFootstepState(ROS2SyncedRobotModel syncedRobot,
                                          FootstepPlanActionState footstepPlan,
                                          FootstepPlanActionFootstepDefinition definition)
   {
      this.syncedRobot = syncedRobot;
      this.footstepPlan = footstepPlan;
      this.definition = definition;

      soleFrame = new DetachableReferenceFrame(definition.getSoleToPlanFrameTransform());
   }

   public void update()
   {
      soleFrame.update(footstepPlan.getDefinition().getParentFrameName(), syncedRobot.getReferenceFrames().getCommonReferenceFrames());
   }

   public void toMessage(FootstepPlanActionFootstepStateMessage message)
   {
      message.setIndex(index);
   }

   public void fromMessage(FootstepPlanActionFootstepStateMessage message)
   {
      index = message.getIndex();
   }

   public FootstepPlanActionFootstepDefinition getDefinition()
   {
      return definition;
   }

   public DetachableReferenceFrame getSoleFrame()
   {
      return soleFrame;
   }

   public int getIndex()
   {
      return index;
   }

   public void setIndex(int index)
   {
      this.index = index;
   }
}
