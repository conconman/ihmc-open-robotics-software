package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MultiContactCoMRegionMessage" defined in "MultiContactCoMRegionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MultiContactCoMRegionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MultiContactCoMRegionMessage_.idl instead.
*
*/
public class MultiContactCoMRegionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.MultiContactCoMRegionMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::MultiContactCoMRegionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "37922f872bbeedf1264f7832ed8aa694ac40d840737532c00736532e38408781";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.MultiContactCoMRegionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.MultiContactCoMRegionMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 20; ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.Point2DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.MultiContactCoMRegionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.MultiContactCoMRegionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getSupportPolygon().size(); ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.Point2DMessagePubSubType.getCdrSerializedSize(data.getSupportPolygon().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.MultiContactCoMRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getSupportPolygon().size() <= 20)
      cdr.write_type_e(data.getSupportPolygon());else
          throw new RuntimeException("support_polygon field exceeds the maximum length");

   }

   public static void read(toolbox_msgs.msg.dds.MultiContactCoMRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getSupportPolygon());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.MultiContactCoMRegionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("support_polygon", data.getSupportPolygon());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.MultiContactCoMRegionMessage data)
   {
      ser.read_type_e("support_polygon", data.getSupportPolygon());
   }

   public static void staticCopy(toolbox_msgs.msg.dds.MultiContactCoMRegionMessage src, toolbox_msgs.msg.dds.MultiContactCoMRegionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.MultiContactCoMRegionMessage createData()
   {
      return new toolbox_msgs.msg.dds.MultiContactCoMRegionMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.MultiContactCoMRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.MultiContactCoMRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.MultiContactCoMRegionMessage src, toolbox_msgs.msg.dds.MultiContactCoMRegionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MultiContactCoMRegionMessagePubSubType newInstance()
   {
      return new MultiContactCoMRegionMessagePubSubType();
   }
}
