package us.ihmc.behaviors.sequence.actions.footstep;

import behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage;
import behavior_msgs.msg.dds.FootstepPlanActionStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.lists.RecyclingArrayListTools;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;

public class FootstepPlanActionState extends BehaviorActionState
{
   private ROS2SyncedRobotModel syncedRobot;
   private final FootstepPlanActionDefinition definition = new FootstepPlanActionDefinition();
   private int numberOfAllocatedFootsteps = 0;
   private final RecyclingArrayList<FootstepPlanActionFootstepState> footsteps;
   private final DetachableReferenceFrame parentFrame;

   public FootstepPlanActionState(ROS2SyncedRobotModel syncedRobot)
   {
      this.syncedRobot = syncedRobot;
      footsteps = new RecyclingArrayList<>(() ->
         new FootstepPlanActionFootstepState(syncedRobot,
                                             this,
                                             RecyclingArrayListTools.getUnsafe(definition.getFootsteps(), numberOfAllocatedFootsteps++)));
      parentFrame = new DetachableReferenceFrame();
   }

   @Override
   public void update()
   {
      RecyclingArrayListTools.synchronizeSize(footsteps, definition.getFootsteps());

      for (int i = 0; i < footsteps.size(); i++)
      {
         footsteps.get(i).setIndex(i);
         footsteps.get(i).update();
      }

      parentFrame.update(definition.getParentFrameName(), syncedRobot.getReferenceFrames().getCommonReferenceFrames());

      setCanExecute(parentFrame.isChildOfWorld());
   }

   public void toMessage(FootstepPlanActionStateMessage message)
   {
      super.toMessage(message.getActionState());

      definition.toMessage(message.getDefinition());

      message.getFootsteps().clear();
      for (FootstepPlanActionFootstepState footstep : footsteps)
      {
         footstep.toMessage(message.getFootsteps().add());
      }
   }

   public void fromMessage(FootstepPlanActionStateMessage message)
   {
      super.fromMessage(message.getActionState());

      definition.fromMessage(message.getDefinition());

      footsteps.clear();
      for (FootstepPlanActionFootstepStateMessage footstep : message.getFootsteps())
      {
         footsteps.add().fromMessage(footstep);
      }
   }

   public RecyclingArrayList<FootstepPlanActionFootstepState> getFootsteps()
   {
      return footsteps;
   }

   @Override
   public FootstepPlanActionDefinition getDefinition()
   {
      return definition;
   }

   public DetachableReferenceFrame getParentFrame()
   {
      return parentFrame;
   }
}
