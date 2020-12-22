package us.ihmc.humanoidBehaviors.ui.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import controller_msgs.msg.dds.LidarScanMessage;
import org.lwjgl.opengl.GL30;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.GDX3DApplication;
import us.ihmc.gdx.GDXApplicationCreator;
import us.ihmc.gdx.GDXModelPrimitives;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.SingleThreadSizeOneQueueExecutor;

import java.util.concurrent.atomic.AtomicReference;

public class GDXROS2PointCloudViewer extends GDX3DApplication
{
   private final ROS2Node ros2Node;
   private final SingleThreadSizeOneQueueExecutor threadQueue;

   private AtomicReference<Point3D32[]> points = new AtomicReference<>();

   public GDXROS2PointCloudViewer(ROS2Node ros2Node, ROS2Topic<?> topic)
   {
      this.ros2Node = ros2Node;
      threadQueue = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

      if (topic.getType().equals(LidarScanMessage.class))
      {
         new IHMCROS2Callback<>(ros2Node, topic.withType(LidarScanMessage.class), this::queueRenderLidarScan);
      }
//      else if (topic.getType().equals(StereoVisionPointCloudMessage.class))
//      {
//         new IHMCROS2Callback<>(ros2Node, topic.withType(StereoVisionPointCloudMessage.class), this::queueRenderStereoVisionPointCloud);
//      }

      Lwjgl3Application lwjgl3App = GDXApplicationCreator.launchGDXApplication(new PrivateGDX3DApplication(), "GDX3DDemo", 1100, 800);
   }

//   private void queueRenderStereoVisionPointCloud(StereoVisionPointCloudMessage message)
//   {
//      threadQueue.submitTask(() ->
//      {
//         Point3D32[] points = PointCloudCompression.decompressPointCloudToArray32(message);
//         int[] colors = PointCloudCompression.decompressColorsToIntArray(message);
//
//         this.points.set(points);
////         buildMesh(points, colors);
//      });
//   }

   private void queueRenderLidarScan(LidarScanMessage message)
   {
      threadQueue.submitTask(() ->
      {
         int numberOfPoints = message.getScan().size() / 3;
         Point3D32[] points = new Point3D32[numberOfPoints];
         int[] colors = new int[numberOfPoints];

         for (int i = 0; i < numberOfPoints; i++)
         {
            points[i] = new Point3D32();
            points[i].setX(message.getScan().get(i * 3));
            points[i].setY(message.getScan().get(i * 3 + 1));
            points[i].setZ(message.getScan().get(i * 3 + 2));

            colors[i] = 0;
         }

         this.points.set(points);
//         buildMesh(points, colors);
      });
   }

   class PrivateGDX3DApplication extends GDX3DApplication
   {
      private Renderable renderable;
      private float[] vertices;
      
      private final int sizeAndRotationUsage = 1 << 9;
      private final VertexAttributes CPU_ATTRIBUTES = new VertexAttributes(
            new VertexAttribute(VertexAttributes.Usage.Position, 3, ShaderProgram.POSITION_ATTRIBUTE),
            new VertexAttribute(VertexAttributes.Usage.ColorUnpacked, 4,ShaderProgram.COLOR_ATTRIBUTE),
            new VertexAttribute(sizeAndRotationUsage, 3, "a_sizeAndRotation")
      );
      private final int CPU_VERTEX_SIZE = (short) (CPU_ATTRIBUTES.vertexSize / 4);
      private final int CPU_POSITION_OFFSET = (short) (CPU_ATTRIBUTES.findByUsage(VertexAttributes.Usage.Position).offset / 4);
      private final int CPU_COLOR_OFFSET = (short) (CPU_ATTRIBUTES.findByUsage(VertexAttributes.Usage.ColorUnpacked).offset / 4);
      private final int CPU_SIZE_AND_ROTATION_OFFSET = (short) (CPU_ATTRIBUTES.findByUsage(sizeAndRotationUsage).offset / 4);

      @Override
      public void create()
      {
         super.create();

         addModelInstance(new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3)));

         renderable = new Renderable();
         renderable.meshPart.primitiveType = GL20.GL_POINTS;
         renderable.meshPart.offset = 0;
         renderable.material = new Material(ColorAttribute.createDiffuse(Color.WHITE));

         int capacity = 1000;
         vertices = new float[capacity * CPU_VERTEX_SIZE];
         if (renderable.meshPart.mesh != null) renderable.meshPart.mesh.dispose();
         renderable.meshPart.mesh = new Mesh(false, capacity, 0, CPU_ATTRIBUTES);
      }

      @Override
      public void render()
      {
         Gdx.gl.glEnable(GL20.GL_VERTEX_PROGRAM_POINT_SIZE);
         Gdx.gl.glEnable(GL30.GL_POINT_SPRITE);

         Point3D32[] pointsToRender = points.get();

         renderBefore();
         renderRegisteredObjects();

         if (pointsToRender != null)
         {
            for (int i = 0; i < pointsToRender.length; i++)
            {
               int offset = i * CPU_VERTEX_SIZE;

               vertices[offset + CPU_POSITION_OFFSET] = pointsToRender[i].getX32();
               vertices[offset + CPU_POSITION_OFFSET + 1] = pointsToRender[i].getY32();
               vertices[offset + CPU_POSITION_OFFSET + 2] = pointsToRender[i].getZ32();
               vertices[offset + CPU_COLOR_OFFSET] = Color.WHITE.r;
               vertices[offset + CPU_COLOR_OFFSET + 1] = Color.WHITE.g;
               vertices[offset + CPU_COLOR_OFFSET + 2] = Color.WHITE.b;
               vertices[offset + CPU_COLOR_OFFSET + 3] = Color.WHITE.a;
               vertices[offset + CPU_SIZE_AND_ROTATION_OFFSET] = 1;
               vertices[offset + CPU_SIZE_AND_ROTATION_OFFSET + 1] = 0;
               vertices[offset + CPU_SIZE_AND_ROTATION_OFFSET + 2] = 1;
            }

            renderable.meshPart.size = pointsToRender.length;
            renderable.meshPart.mesh.setVertices(vertices, 0, pointsToRender.length * CPU_VERTEX_SIZE);
            renderable.meshPart.update();

            getModelBatch().render(renderable);
         }

         renderAfter();
      }
   }

   public static void main(String[] args)
   {
      new GDXROS2PointCloudViewer(ROS2Tools.createInterprocessROS2Node("point_cloud_viewer"), ROS2Tools.MULTISENSE_LIDAR_SCAN);
   }
}
