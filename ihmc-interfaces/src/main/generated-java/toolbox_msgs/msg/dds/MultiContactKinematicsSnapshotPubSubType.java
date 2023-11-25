package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MultiContactKinematicsSnapshot" defined in "MultiContactKinematicsSnapshot_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MultiContactKinematicsSnapshot_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MultiContactKinematicsSnapshot_.idl instead.
*
*/
public class MultiContactKinematicsSnapshotPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::MultiContactKinematicsSnapshot_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "dd54154fe488672bc8b707014096920c307de460d593a9b82b2a7c5914d4a582";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot data) throws java.io.IOException
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

      current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (20 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 20; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessagePubSubType.getCdrSerializedSize(data.getKinematicsSnapshot(), current_alignment);

      current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.getCdrSerializedSize(data.getKinematicsSolution(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getInContact().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getSurfaceNormals().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getSurfaceNormals().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot data, us.ihmc.idl.CDR cdr)
   {
      toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessagePubSubType.write(data.getKinematicsSnapshot(), cdr);
      toolbox_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.write(data.getKinematicsSolution(), cdr);
      if(data.getInContact().size() <= 20)
      cdr.write_type_e(data.getInContact());else
          throw new RuntimeException("in_contact field exceeds the maximum length");

      if(data.getSurfaceNormals().size() <= 20)
      cdr.write_type_e(data.getSurfaceNormals());else
          throw new RuntimeException("surface_normals field exceeds the maximum length");

   }

   public static void read(toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot data, us.ihmc.idl.CDR cdr)
   {
      toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessagePubSubType.read(data.getKinematicsSnapshot(), cdr);	
      toolbox_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.read(data.getKinematicsSolution(), cdr);	
      cdr.read_type_e(data.getInContact());	
      cdr.read_type_e(data.getSurfaceNormals());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("kinematics_snapshot", new toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessagePubSubType(), data.getKinematicsSnapshot());

      ser.write_type_a("kinematics_solution", new toolbox_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType(), data.getKinematicsSolution());

      ser.write_type_e("in_contact", data.getInContact());
      ser.write_type_e("surface_normals", data.getSurfaceNormals());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot data)
   {
      ser.read_type_a("kinematics_snapshot", new toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessagePubSubType(), data.getKinematicsSnapshot());

      ser.read_type_a("kinematics_solution", new toolbox_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType(), data.getKinematicsSolution());

      ser.read_type_e("in_contact", data.getInContact());
      ser.read_type_e("surface_normals", data.getSurfaceNormals());
   }

   public static void staticCopy(toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot src, toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot createData()
   {
      return new toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot();
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
   
   public void serialize(toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot src, toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MultiContactKinematicsSnapshotPubSubType newInstance()
   {
      return new MultiContactKinematicsSnapshotPubSubType();
   }
}
