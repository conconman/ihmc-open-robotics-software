package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlanActionStateBasicsMessage" defined in "FootstepPlanActionStateBasicsMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlanActionStateBasicsMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlanActionStateBasicsMessage_.idl instead.
*
*/
public class FootstepPlanActionStateBasicsMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::FootstepPlanActionStateBasicsMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "118a8608935a90a8e8946620bc8540790aa2d1dbfe707d91ee6cfd2204c62e7b";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getDesiredLeftFootsteps().size(); ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType.getCdrSerializedSize(data.getDesiredLeftFootsteps().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getDesiredRightFootsteps().size(); ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType.getCdrSerializedSize(data.getDesiredRightFootsteps().get(i0), current_alignment);}

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getCurrentLeftFootPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getCurrentRightFootPose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_3(data.getTotalNumberOfFootsteps());

      cdr.write_type_3(data.getNumberOfIncompleteFootsteps());

      if(data.getDesiredLeftFootsteps().size() <= 50)
      cdr.write_type_e(data.getDesiredLeftFootsteps());else
          throw new RuntimeException("desired_left_footsteps field exceeds the maximum length");

      if(data.getDesiredRightFootsteps().size() <= 50)
      cdr.write_type_e(data.getDesiredRightFootsteps());else
          throw new RuntimeException("desired_right_footsteps field exceeds the maximum length");

      geometry_msgs.msg.dds.PosePubSubType.write(data.getCurrentLeftFootPose(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getCurrentRightFootPose(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setTotalNumberOfFootsteps(cdr.read_type_3());
      	
      data.setNumberOfIncompleteFootsteps(cdr.read_type_3());
      	
      cdr.read_type_e(data.getDesiredLeftFootsteps());	
      cdr.read_type_e(data.getDesiredRightFootsteps());	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getCurrentLeftFootPose(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getCurrentRightFootPose(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_3("total_number_of_footsteps", data.getTotalNumberOfFootsteps());
      ser.write_type_3("number_of_incomplete_footsteps", data.getNumberOfIncompleteFootsteps());
      ser.write_type_e("desired_left_footsteps", data.getDesiredLeftFootsteps());
      ser.write_type_e("desired_right_footsteps", data.getDesiredRightFootsteps());
      ser.write_type_a("current_left_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getCurrentLeftFootPose());

      ser.write_type_a("current_right_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getCurrentRightFootPose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage data)
   {
      data.setTotalNumberOfFootsteps(ser.read_type_3("total_number_of_footsteps"));
      data.setNumberOfIncompleteFootsteps(ser.read_type_3("number_of_incomplete_footsteps"));
      ser.read_type_e("desired_left_footsteps", data.getDesiredLeftFootsteps());
      ser.read_type_e("desired_right_footsteps", data.getDesiredRightFootsteps());
      ser.read_type_a("current_left_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getCurrentLeftFootPose());

      ser.read_type_a("current_right_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getCurrentRightFootPose());

   }

   public static void staticCopy(behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage src, behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage createData()
   {
      return new behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage();
   }
   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage src, behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlanActionStateBasicsMessagePubSubType newInstance()
   {
      return new FootstepPlanActionStateBasicsMessagePubSubType();
   }
}
