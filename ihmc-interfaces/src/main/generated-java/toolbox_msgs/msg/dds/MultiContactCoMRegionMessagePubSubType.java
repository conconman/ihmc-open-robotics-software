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
   		return "a233aa4194a35fd929e9065b5bfac052ffa8ef23581aa5ab883d4a12656fe029";
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

      current_alignment += controller_msgs.msg.dds.Polygon2DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 15; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.Polygon2DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.MultiContactCoMRegionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.MultiContactCoMRegionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += controller_msgs.msg.dds.Polygon2DMessagePubSubType.getCdrSerializedSize(data.getComFeasibilityRegion(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getComFeasibilityPreviewRegions().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.Polygon2DMessagePubSubType.getCdrSerializedSize(data.getComFeasibilityPreviewRegions().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.MultiContactCoMRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      controller_msgs.msg.dds.Polygon2DMessagePubSubType.write(data.getComFeasibilityRegion(), cdr);
      if(data.getComFeasibilityPreviewRegions().size() <= 15)
      cdr.write_type_e(data.getComFeasibilityPreviewRegions());else
          throw new RuntimeException("com_feasibility_preview_regions field exceeds the maximum length");

   }

   public static void read(toolbox_msgs.msg.dds.MultiContactCoMRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      controller_msgs.msg.dds.Polygon2DMessagePubSubType.read(data.getComFeasibilityRegion(), cdr);	
      cdr.read_type_e(data.getComFeasibilityPreviewRegions());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.MultiContactCoMRegionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("com_feasibility_region", new controller_msgs.msg.dds.Polygon2DMessagePubSubType(), data.getComFeasibilityRegion());

      ser.write_type_e("com_feasibility_preview_regions", data.getComFeasibilityPreviewRegions());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.MultiContactCoMRegionMessage data)
   {
      ser.read_type_a("com_feasibility_region", new controller_msgs.msg.dds.Polygon2DMessagePubSubType(), data.getComFeasibilityRegion());

      ser.read_type_e("com_feasibility_preview_regions", data.getComFeasibilityPreviewRegions());
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
