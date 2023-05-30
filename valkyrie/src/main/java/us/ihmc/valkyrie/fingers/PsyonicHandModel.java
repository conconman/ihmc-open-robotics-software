package us.ihmc.valkyrie.fingers;

import com.google.common.base.CaseFormat;

import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class PsyonicHandModel implements HandModel
{
   @Override
   public PsyonicJointName[] getHandJointNames()
   {
      return PsyonicJointName.values;
   }

   public static enum PsyonicJointName implements HandJointName
   {
      index_q1, index_q2, middle_q1, middle_q2, ring_q1, ring_q2, pinky_q1, pinky_q2, thumb_q1, thumb_q2;

      public static final PsyonicJointName[] values = values();

      @Override
      public String getJointName(RobotSide robotSide)
      {
         return name() + "_" + robotSide.getCamelCaseName();
      }

      public String getCamelCaseJointName(RobotSide side)
      {
         return side.getCamelCaseName() + CaseFormat.LOWER_UNDERSCORE.to(CaseFormat.UPPER_CAMEL, name());
      }

      public String getPascalCaseJointName(RobotSide side)
      {
         return side.getPascalCaseName() + CaseFormat.LOWER_UNDERSCORE.to(CaseFormat.UPPER_CAMEL, name());
      }

      @Override
      public int getIndex(RobotSide robotSide)
      {
         return ordinal();
      }
   }
}
