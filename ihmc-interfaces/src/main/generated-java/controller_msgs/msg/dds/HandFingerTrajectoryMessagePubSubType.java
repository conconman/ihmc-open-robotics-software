package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandFingerTrajectoryMessage" defined in "HandFingerTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandFingerTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandFingerTrajectoryMessage_.idl instead.
*
*/
public class HandFingerTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.HandFingerTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::HandFingerTrajectoryMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.HandFingerTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.HandFingerTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.OneDoFJointTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.QueueableMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandFingerTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandFingerTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getJointTrajectoryMessages().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.OneDoFJointTrajectoryMessagePubSubType.getCdrSerializedSize(data.getJointTrajectoryMessages().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getListQueueingProperties().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.QueueableMessagePubSubType.getCdrSerializedSize(data.getListQueueingProperties().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.HandFingerTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      if(data.getJointTrajectoryMessages().size() <= 100)
      cdr.write_type_e(data.getJointTrajectoryMessages());else
          throw new RuntimeException("joint_trajectory_messages field exceeds the maximum length");

      if(data.getListQueueingProperties().size() <= 100)
      cdr.write_type_e(data.getListQueueingProperties());else
          throw new RuntimeException("list_queueing_properties field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.HandFingerTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      cdr.read_type_e(data.getJointTrajectoryMessages());	
      cdr.read_type_e(data.getListQueueingProperties());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.HandFingerTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_e("joint_trajectory_messages", data.getJointTrajectoryMessages());
      ser.write_type_e("list_queueing_properties", data.getListQueueingProperties());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.HandFingerTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_e("joint_trajectory_messages", data.getJointTrajectoryMessages());
      ser.read_type_e("list_queueing_properties", data.getListQueueingProperties());
   }

   public static void staticCopy(controller_msgs.msg.dds.HandFingerTrajectoryMessage src, controller_msgs.msg.dds.HandFingerTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.HandFingerTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.HandFingerTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.HandFingerTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.HandFingerTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.HandFingerTrajectoryMessage src, controller_msgs.msg.dds.HandFingerTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandFingerTrajectoryMessagePubSubType newInstance()
   {
      return new HandFingerTrajectoryMessagePubSubType();
   }
}
