package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MultiContactCoMRegionPreviewMessage" defined in "MultiContactCoMRegionPreviewMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MultiContactCoMRegionPreviewMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MultiContactCoMRegionPreviewMessage_.idl instead.
*
*/
public class MultiContactCoMRegionPreviewMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::MultiContactCoMRegionPreviewMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ad31cc4e41f3504083cd3c8fe4825fd9bae1055a1032a2a11fb9794d3b3551e2";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 15; ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.MultiContactCoMRegionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getComPreviewRegions().size(); ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.MultiContactCoMRegionMessagePubSubType.getCdrSerializedSize(data.getComPreviewRegions().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getComPreviewRegions().size() <= 15)
      cdr.write_type_e(data.getComPreviewRegions());else
          throw new RuntimeException("com_preview_regions field exceeds the maximum length");

   }

   public static void read(toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getComPreviewRegions());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("com_preview_regions", data.getComPreviewRegions());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage data)
   {
      ser.read_type_e("com_preview_regions", data.getComPreviewRegions());
   }

   public static void staticCopy(toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage src, toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage createData()
   {
      return new toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage src, toolbox_msgs.msg.dds.MultiContactCoMRegionPreviewMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MultiContactCoMRegionPreviewMessagePubSubType newInstance()
   {
      return new MultiContactCoMRegionPreviewMessagePubSubType();
   }
}
