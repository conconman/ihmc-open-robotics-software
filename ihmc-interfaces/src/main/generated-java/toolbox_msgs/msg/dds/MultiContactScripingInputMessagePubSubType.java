package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MultiContactScripingInputMessage" defined in "MultiContactScripingInputMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MultiContactScripingInputMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MultiContactScripingInputMessage_.idl instead.
*
*/
public class MultiContactScripingInputMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.MultiContactScripingInputMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::MultiContactScripingInputMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "29f72f2dc76a52ae64fd5999da2fa91900ab685ddfa8e92e8a236c6097ad4326";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.MultiContactScripingInputMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.MultiContactScripingInputMessage data) throws java.io.IOException
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

      current_alignment += toolbox_msgs.msg.dds.MultiContactKinematicsSnapshotPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += toolbox_msgs.msg.dds.MultiContactKinematicsSnapshotPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.MultiContactScripingInputMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.MultiContactScripingInputMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += toolbox_msgs.msg.dds.MultiContactKinematicsSnapshotPubSubType.getCdrSerializedSize(data.getSnapshot(), current_alignment);

      current_alignment += toolbox_msgs.msg.dds.MultiContactKinematicsSnapshotPubSubType.getCdrSerializedSize(data.getPreviousSnapshot(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.MultiContactScripingInputMessage data, us.ihmc.idl.CDR cdr)
   {
      toolbox_msgs.msg.dds.MultiContactKinematicsSnapshotPubSubType.write(data.getSnapshot(), cdr);
      toolbox_msgs.msg.dds.MultiContactKinematicsSnapshotPubSubType.write(data.getPreviousSnapshot(), cdr);
      cdr.write_type_7(data.getContainsPreviousKeyframe());

   }

   public static void read(toolbox_msgs.msg.dds.MultiContactScripingInputMessage data, us.ihmc.idl.CDR cdr)
   {
      toolbox_msgs.msg.dds.MultiContactKinematicsSnapshotPubSubType.read(data.getSnapshot(), cdr);	
      toolbox_msgs.msg.dds.MultiContactKinematicsSnapshotPubSubType.read(data.getPreviousSnapshot(), cdr);	
      data.setContainsPreviousKeyframe(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.MultiContactScripingInputMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("snapshot", new toolbox_msgs.msg.dds.MultiContactKinematicsSnapshotPubSubType(), data.getSnapshot());

      ser.write_type_a("previous_snapshot", new toolbox_msgs.msg.dds.MultiContactKinematicsSnapshotPubSubType(), data.getPreviousSnapshot());

      ser.write_type_7("contains_previous_keyframe", data.getContainsPreviousKeyframe());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.MultiContactScripingInputMessage data)
   {
      ser.read_type_a("snapshot", new toolbox_msgs.msg.dds.MultiContactKinematicsSnapshotPubSubType(), data.getSnapshot());

      ser.read_type_a("previous_snapshot", new toolbox_msgs.msg.dds.MultiContactKinematicsSnapshotPubSubType(), data.getPreviousSnapshot());

      data.setContainsPreviousKeyframe(ser.read_type_7("contains_previous_keyframe"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.MultiContactScripingInputMessage src, toolbox_msgs.msg.dds.MultiContactScripingInputMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.MultiContactScripingInputMessage createData()
   {
      return new toolbox_msgs.msg.dds.MultiContactScripingInputMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.MultiContactScripingInputMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.MultiContactScripingInputMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.MultiContactScripingInputMessage src, toolbox_msgs.msg.dds.MultiContactScripingInputMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MultiContactScripingInputMessagePubSubType newInstance()
   {
      return new MultiContactScripingInputMessagePubSubType();
   }
}
