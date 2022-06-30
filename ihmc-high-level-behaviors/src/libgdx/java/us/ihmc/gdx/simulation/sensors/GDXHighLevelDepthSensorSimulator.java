package us.ihmc.gdx.simulation.sensors;

import boofcv.struct.calib.CameraPinholeBrown;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.BigVideoPacket;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import org.apache.commons.lang3.mutable.MutableInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.Time;
import sensor_msgs.Image;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.StereoPointCloudCompression;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.sceneManager.GDX3DScene;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudMessageTools;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.publisher.RosCameraInfoPublisher;
import us.ihmc.utilities.ros.publisher.RosImagePublisher;
import us.ihmc.utilities.ros.publisher.RosPointCloudPublisher;
import us.ihmc.utilities.ros.types.PointType;

import java.nio.ByteBuffer;
import java.time.Instant;
import java.util.Arrays;
import java.util.function.LongSupplier;

public class GDXHighLevelDepthSensorSimulator extends ImGuiPanel implements RenderableProvider
{
   private static final MutableInt INDEX = new MutableInt();
   private final String sensorName;
   private final ReferenceFrame sensorFrame;
   private final Matrix4 gdxTransform = new Matrix4();
   private final GDXLowLevelDepthSensorSimulator depthSensorSimulator;
   private final LongSupplier timestampSupplier;
   private final CameraPinholeBrown depthCameraIntrinsics;
   private final int imageWidth;
   private final int imageHeight;
   private final GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();
   private final Mat rgba8Mat;

   private RosNodeInterface ros1Node;
   private String ros1DepthImageTopic;
   private String ros1ColorImageTopic;
   private ROS2Topic<?> ros2PointCloudTopic;
   private String ros1PointCloudTopic;
   private RosImagePublisher ros1DepthPublisher;
   private RosPointCloudPublisher ros1PointCloudPublisher;
   private RosCameraInfoPublisher ros1DepthCameraInfoPublisher;
   private ChannelBuffer ros1DepthChannelBuffer;
   private RosImagePublisher ros1ColorPublisher;
   private RosCameraInfoPublisher ros1ColorCameraInfoPublisher;
   private ChannelBuffer ros1ColorChannelBuffer;
   private Mat rgb8Mat;
   private ByteBuffer rgb8Buffer;

   private ROS2NodeInterface ros2Node;
   private boolean ros2IsLidarScan;
   private IHMCROS2Publisher<?> publisher;
   private RealtimeROS2Node realtimeROS2Node;
   private IHMCRealtimeROS2Publisher<BigVideoPacket> ros2VideoPublisher;
   private BigVideoPacket videoPacket;
   private BytePointer jpegImageBytePointer;
   private Mat yuv420Image;
   private IntPointer compressionParameters;
   private RecyclingArrayList<Point3D> ros1PointsToPublish;
   private RecyclingArrayList<Point3D> ros2PointsToPublish;
   private int[] ros2ColorsToPublish;
   private final FramePose3D tempSensorFramePose = new FramePose3D();

   private final Timer throttleTimer = new Timer();
   private final ResettableExceptionHandlingExecutorService depthExecutor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ResettableExceptionHandlingExecutorService colorExecutor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ResettableExceptionHandlingExecutorService pointCloudExecutor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ResettableExceptionHandlingExecutorService colorROS2Executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final double publishRateHz;
   private final ImBoolean debugCoordinateFrame = new ImBoolean(false);
   private ModelInstance coordinateFrame;
   private RigidBodyTransform sensorFrameToWorldTransform;

   private boolean tuning = false;
   private final ImBoolean sensorEnabled = new ImBoolean(false);
   private final ImBoolean renderPointCloudDirectly = new ImBoolean(false);
   private final ImBoolean publishDepthImageROS1 = new ImBoolean(false);
   private final ImBoolean publishColorImageROS1 = new ImBoolean(false);
   private final ImBoolean publishPointCloudROS1 = new ImBoolean(false);
   private final ImBoolean publishColorImageROS2 = new ImBoolean(false);
   private final ImBoolean publishPointCloudROS2 = new ImBoolean(false);

   private final ImBoolean useSensorColor = new ImBoolean(false);
   private final Color pointColorFromPicker = new Color();
   private final ImFloat pointSize = new ImFloat(0.01f);
   private final float[] color = new float[] {1.0f, 1.0f, 1.0f, 1.0f};

   public GDXHighLevelDepthSensorSimulator(String sensorName,
                                           ReferenceFrame sensorFrame,
                                           LongSupplier timestampSupplier,
                                           double verticalFOV,
                                           int imageWidth,
                                           int imageHeight,
                                           double minRange,
                                           double maxRange,
                                           double noiseAmplitudeAtMinRange,
                                           double noiseAmplitudeAtMaxRange,
                                           boolean simulateL515Noise,
                                           double publishRateHz)
   {
      super(ImGuiTools.uniqueLabel(INDEX.getAndIncrement(), sensorName + " Simulator"));
      this.sensorName = sensorName;
      setRenderMethod(this::renderImGuiWidgets);
      depthSensorSimulator = new GDXLowLevelDepthSensorSimulator(sensorName,
                                                                 verticalFOV,
                                                                 imageWidth,
                                                                 imageHeight,
                                                                 minRange,
                                                                 maxRange,
                                                                 noiseAmplitudeAtMinRange,
                                                                 noiseAmplitudeAtMaxRange,
                                                                 simulateL515Noise);

      this.sensorFrame = sensorFrame;
      this.timestampSupplier = timestampSupplier;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.publishRateHz = publishRateHz;

      pointCloudRenderer.create(imageWidth * imageHeight);

      depthSensorSimulator.create(pointCloudRenderer.getVertexBuffer());
      addChild(depthSensorSimulator.getDepthPanel());
      addChild(depthSensorSimulator.getColorPanel());

      if (debugCoordinateFrame.get())
         coordinateFrame = GDXModelBuilder.createCoordinateFrameInstance(0.2);

      depthCameraIntrinsics = new CameraPinholeBrown();
      updateCameraPinholeBrown();

      rgba8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, new BytePointer(depthSensorSimulator.getColorRGBA8Buffer()));

      throttleTimer.reset();
   }

   public void setupForROS1Depth(RosNodeInterface ros1Node, String ros1DepthImageTopic, String ros1DepthCameraInfoTopic)
   {
      this.ros1Node = ros1Node;
      this.ros1DepthImageTopic = ros1DepthImageTopic;
      LogTools.info("Publishing ROS 1 depth: {} {}", ros1DepthImageTopic, ros1DepthCameraInfoTopic);
      ros1DepthPublisher = new RosImagePublisher();
      ros1DepthCameraInfoPublisher = new RosCameraInfoPublisher();
      ros1Node.attachPublisher(ros1DepthCameraInfoTopic, ros1DepthCameraInfoPublisher);
      ros1Node.attachPublisher(ros1DepthImageTopic, ros1DepthPublisher);
      ros1DepthChannelBuffer = ros1DepthPublisher.getChannelBufferFactory().getBuffer(2 * imageWidth * imageHeight);
   }

   public void setupForROS1Color(RosNodeInterface ros1Node, String ros1ColorImageTopic, String ros1ColorCameraInfoTopic)
   {
      this.ros1Node = ros1Node;
      this.ros1ColorImageTopic = ros1ColorImageTopic;
      LogTools.info("Publishing ROS 1 color: {} {}", ros1ColorImageTopic, ros1ColorCameraInfoTopic);
      ros1ColorPublisher = new RosImagePublisher();
      ros1ColorCameraInfoPublisher = new RosCameraInfoPublisher();
      ros1Node.attachPublisher(ros1ColorCameraInfoTopic, ros1ColorCameraInfoPublisher);
      ros1Node.attachPublisher(ros1ColorImageTopic, ros1ColorPublisher);
      ros1ColorChannelBuffer = ros1ColorPublisher.getChannelBufferFactory().getBuffer(3 * imageWidth * imageHeight);
      rgb8Buffer = BufferUtils.newByteBuffer(imageWidth * imageHeight * 3);
      rgb8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC3, new BytePointer(rgb8Buffer));
   }

   public void setupForROS1PointCloud(RosNodeInterface ros1Node, String ros1PointCloudTopic)
   {
      this.ros1Node = ros1Node;
      this.ros1PointCloudTopic = ros1PointCloudTopic;
      ros1PointsToPublish = new RecyclingArrayList<>(imageWidth * imageHeight, Point3D::new);
      ros1PointCloudPublisher = new RosPointCloudPublisher(PointType.XYZ, false);
      ros1Node.attachPublisher(ros1PointCloudTopic, ros1PointCloudPublisher);
   }

   public void setupForROS2PointCloud(ROS2NodeInterface ros2Node, ROS2Topic<?> ros2PointCloudTopic)
   {
      this.ros2Node = ros2Node;
      this.ros2PointCloudTopic = ros2PointCloudTopic;
      ros2PointsToPublish = new RecyclingArrayList<>(imageWidth * imageHeight, Point3D::new);
      ros2IsLidarScan = ros2PointCloudTopic.getType().equals(LidarScanMessage.class);
      if (!ros2IsLidarScan)
         ros2ColorsToPublish = new int[imageWidth * imageHeight];
      LogTools.info("Publishing ROS 2 point cloud: {}", ros2PointCloudTopic.getName());
      publisher = ROS2Tools.createPublisher(ros2Node, ros2PointCloudTopic, ROS2QosProfile.DEFAULT());
   }

   public void setupForROS2Color(PubSubImplementation pubSubImplementation, ROS2Topic<BigVideoPacket> ros2VideoTopic)
   {
      // A Realtime ROS 2 node is required for video streaming in order to get stable performance.
      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(sensorName) + "_video");
      ros2VideoPublisher = ROS2Tools.createPublisher(realtimeROS2Node, ros2VideoTopic, ROS2QosProfile.BEST_EFFORT());
      realtimeROS2Node.spin();
      videoPacket = new BigVideoPacket();
      yuv420Image = new Mat();
      jpegImageBytePointer = new BytePointer();
      compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75);
   }

   public void render(GDX3DScene scene)
   {
      if (sensorEnabled.get())
      {
         if (sensorFrame != null)
            GDXTools.toGDX(sensorFrame.getTransformToWorldFrame(), gdxTransform);
         else
            GDXTools.toGDX(sensorFrameToWorldTransform, gdxTransform);

         depthSensorSimulator.setCameraWorldTransform(gdxTransform);

         if (coordinateFrame != null)
            coordinateFrame.transform.set(gdxTransform);

         if (renderPointCloudDirectly.get())
         {
            GDXTools.toGDX(color, pointColorFromPicker);
            Color pointColor = useSensorColor.get() ? null : pointColorFromPicker;
            depthSensorSimulator.render(scene, pointColor, pointSize.get());
            pointCloudRenderer.updateMeshFastest(imageWidth * imageHeight);
         }
         else
         {
            depthSensorSimulator.render(scene);
         }
         if (throttleTimer.isExpired(UnitConversions.hertzToSeconds(publishRateHz)))
         {
            if (ros1Node != null && ros1Node.isStarted())
            {
               if (publishDepthImageROS1.get())
                  publishDepthImageROS1();
               if (publishColorImageROS1.get())
                  publishColorImageROS1();
               if (publishPointCloudROS1.get())
                  publishPointCloudROS1();
            }
            if (ros2Node != null)
            {
               if (publishPointCloudROS2.get())
                  publishPointCloudROS2();
            }
            if (realtimeROS2Node != null)
            {
               if (publishColorImageROS2.get())
                  publishColorImageROS2();
            }
            throttleTimer.reset();
         }

         tuning = false;
      }
   }

   private void publishColorImageROS1()
   {
      if (ros1ColorPublisher.isConnected() && ros1ColorCameraInfoPublisher.isConnected() && !colorExecutor.isExecuting())
      {
         opencv_imgproc.cvtColor(rgba8Mat, rgb8Mat, opencv_imgproc.COLOR_RGBA2RGB);

         ros1ColorChannelBuffer.clear();
         rgb8Buffer.rewind();
         ros1ColorChannelBuffer.writeBytes(rgb8Buffer);

         colorExecutor.execute(() ->
         {
            Image message = ros1ColorPublisher.createMessage(imageWidth, imageHeight, 3, "rgb8", ros1ColorChannelBuffer);

            if (timestampSupplier != null)
               message.getHeader().setStamp(new Time(Conversions.nanosecondsToSeconds(timestampSupplier.getAsLong())));

            ros1ColorPublisher.publish(message);
         });
      }
   }

   private void publishColorImageROS2()
   {
      if (!colorROS2Executor.isExecuting())
      {
         colorROS2Executor.execute(() ->
         {
            long timestamp = timestampSupplier == null ? System.nanoTime() : timestampSupplier.getAsLong();
            Instant now = Instant.now();
            videoPacket.setAcquisitionTimeSecondsSinceEpoch(now.getEpochSecond());
            videoPacket.setAcquisitionTimeAdditionalNanos(now.getNano());

            opencv_imgproc.cvtColor(rgba8Mat, yuv420Image, opencv_imgproc.COLOR_RGBA2YUV_I420);
            opencv_imgcodecs.imencode(".jpg", yuv420Image, jpegImageBytePointer, compressionParameters);

            byte[] heapByteArrayData = new byte[jpegImageBytePointer.asBuffer().remaining()];
            jpegImageBytePointer.asBuffer().get(heapByteArrayData);
            videoPacket.getData().resetQuick();
            videoPacket.getData().add(heapByteArrayData);
            ros2VideoPublisher.publish(videoPacket);
         });
      }
   }

   private void publishDepthImageROS1()
   {
      if (ros1DepthPublisher.isConnected() && ros1DepthCameraInfoPublisher.isConnected() && !depthExecutor.isExecuting())
      {
         PerspectiveCamera camera = depthSensorSimulator.getCamera();
         ByteBuffer depthFloatBuffer = depthSensorSimulator.getMetersDepthFloatBuffer();
         depthFloatBuffer.rewind();
         ros1DepthChannelBuffer.clear();
         int size = 2 * imageWidth * imageHeight;
         for (int y = 0; y < imageHeight; y++)
         {
            for (int x = 0; x < imageWidth; x++)
            {
               float eyeDepthMeters = depthFloatBuffer.getFloat();

               int row = y + 1;
               int backForY = row * imageWidth * 2;
               int forwardForX = x * 2;
               int index = size - backForY + forwardForX;
               if (eyeDepthMeters > camera.near && eyeDepthMeters < depthSensorSimulator.getMaxRange())
               {
                  char depthChar16 = (char) Math.round(eyeDepthMeters * 1000.0f); // 1000 is 1 meter
                  ros1DepthChannelBuffer.setChar(index, depthChar16);
               }
               else
               {
                  ros1DepthChannelBuffer.setChar(index, 0);
               }
            }
         }

         ros1DepthChannelBuffer.readerIndex(0);
         ros1DepthChannelBuffer.writerIndex(size);

         depthExecutor.execute(() ->
         {
            if (tuning)
               updateCameraPinholeBrown();
            ros1DepthCameraInfoPublisher.publish("camera_depth_optical_frame", depthCameraIntrinsics, new Time());
            Image message = ros1DepthPublisher.createMessage(imageWidth, imageHeight, 2, "16UC1", ros1DepthChannelBuffer); // maybe need to copy here if there are errors

            if(timestampSupplier != null)
               message.getHeader().setStamp(new Time(Conversions.nanosecondsToSeconds(timestampSupplier.getAsLong())));

            ros1DepthPublisher.publish(message);
         });
      }
   }

   private void updateCameraPinholeBrown()
   {
      depthCameraIntrinsics.setFx(depthSensorSimulator.getFocalLengthPixels().get());
      depthCameraIntrinsics.setFy(depthSensorSimulator.getFocalLengthPixels().get());
      depthCameraIntrinsics.setSkew(0.0);
      depthCameraIntrinsics.setCx(depthSensorSimulator.getPrincipalOffsetXPixels().get());
      depthCameraIntrinsics.setCy(depthSensorSimulator.getPrincipalOffsetYPixels().get());
   }

   public void renderImGuiWidgets()
   {
      tuning = true;
      ImGui.text("Resolution: " + imageWidth + " x " + imageHeight);
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Sensor Enabled"), sensorEnabled);
      ImGui.sameLine();
      ImGui.checkbox("Show frame graphic", debugCoordinateFrame);
      ImGui.text("Render:");
      ImGui.sameLine();
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Point cloud"), renderPointCloudDirectly);
      ImGui.sameLine();
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Depth video"), getLowLevelSimulator().getDepthPanel().getIsShowing());
      ImGui.sameLine();
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Color video"), getLowLevelSimulator().getColorPanel().getIsShowing());
      ImGui.checkbox("Use Sensor Color", useSensorColor);
      ImGui.sliderFloat("Point size", pointSize.getData(), 0.0001f, 0.10f);
      ImGui.colorPicker4("Color", color);
      ImGui.text("Publish:");
      if (ros1DepthImageTopic != null)
         ImGui.checkbox(ImGuiTools.uniqueLabel(this, "ROS 1 Depth image (" + ros1DepthImageTopic + ")"), publishDepthImageROS1);
      if (ros1ColorImageTopic != null)
         ImGui.checkbox(ImGuiTools.uniqueLabel(this, "ROS 1 Color image (" + ros1ColorImageTopic + ")"), publishColorImageROS1);
      if (ros1PointCloudTopic != null)
         ImGui.checkbox(ImGuiTools.uniqueLabel(this, "ROS 1 Point Cloud (" + ros1PointCloudTopic + ")"), publishPointCloudROS1);
      if (ros2PointCloudTopic != null)
         ImGui.checkbox(ImGuiTools.uniqueLabel(this, "ROS 2 Point cloud (" + ros2PointCloudTopic + ")"), publishPointCloudROS2);
      if (realtimeROS2Node != null)
      {
         ImGui.sameLine();
         ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Color image (ROS 2)"), publishColorImageROS2);
      }
      depthSensorSimulator.renderTuningSliders();
   }

   private void publishPointCloudROS2()
   {
      if (!pointCloudExecutor.isExecuting())
      {
         ros2PointsToPublish.clear();
         for (int i = 0; i < depthSensorSimulator.getNumberOfPoints()
                         && (Float.BYTES * 8 * i + 2) < depthSensorSimulator.getPointCloudBuffer().limit(); i++)
         {
            float x = depthSensorSimulator.getPointCloudBuffer().get(Float.BYTES * 8 * i);
            float y = depthSensorSimulator.getPointCloudBuffer().get(Float.BYTES * 8 * i + 1);
            float z = depthSensorSimulator.getPointCloudBuffer().get(Float.BYTES * 8 * i + 2);
            ros2PointsToPublish.add().set(x, y, z);
            if (ros2ColorsToPublish != null)
               ros2ColorsToPublish[i] = depthSensorSimulator.getColorRGBA8Buffer().getInt(Integer.BYTES * i);
         }

         if (!ros2PointsToPublish.isEmpty())
         {
            pointCloudExecutor.execute(() ->
            {
               long timestamp = timestampSupplier == null ? System.nanoTime() : timestampSupplier.getAsLong();
               tempSensorFramePose.setToZero(sensorFrame);
               tempSensorFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               if (ros2IsLidarScan)
               {
                  LidarScanMessage message = PointCloudMessageTools.toLidarScanMessage(timestamp,
                                                                                       ros2PointsToPublish,
                                                                                       tempSensorFramePose);
                  ((IHMCROS2Publisher<LidarScanMessage>) publisher).publish(message);
               }
               else
               {
                  int size = ros2PointsToPublish.size();
                  Point3D[] points = ros2PointsToPublish.toArray(new Point3D[size]);
                  int[] colors = Arrays.copyOf(ros2ColorsToPublish, size);
                  double minimumResolution = 0.005;
                  StereoVisionPointCloudMessage message = StereoPointCloudCompression.compressPointCloud(timestamp,
                                                                                                         points,
                                                                                                         colors,
                                                                                                         size,
                                                                                                         minimumResolution,
                                                                                                         null);
                  message.getSensorPosition().set(tempSensorFramePose.getPosition());
                  message.getSensorOrientation().set(tempSensorFramePose.getOrientation());
                  //      LogTools.info("Publishing point cloud of size {}", message.getNumberOfPoints());
                  ((IHMCROS2Publisher<StereoVisionPointCloudMessage>) publisher).publish(message);
               }
            });
         }
      }
   }

   private void publishPointCloudROS1()
   {
      if (!pointCloudExecutor.isExecuting())
      {
         ros1PointsToPublish.clear();
         for (int i = 0; i < depthSensorSimulator.getNumberOfPoints()
                         && (Float.BYTES * 8 * i + 2) < depthSensorSimulator.getPointCloudBuffer().limit(); i++)
         {
            float x = depthSensorSimulator.getPointCloudBuffer().get(Float.BYTES * 8 * i);
            float y = depthSensorSimulator.getPointCloudBuffer().get(Float.BYTES * 8 * i + 1);
            float z = depthSensorSimulator.getPointCloudBuffer().get(Float.BYTES * 8 * i + 2);
            ros1PointsToPublish.add().set(x, y, z);
            if (ros2ColorsToPublish != null)
               ros2ColorsToPublish[i] = depthSensorSimulator.getColorRGBA8Buffer().getInt(Integer.BYTES * i);
         }

         if (!ros1PointsToPublish.isEmpty())
         {
            pointCloudExecutor.execute(() ->
            {
//               long timestamp = timestampSupplier == null ? System.nanoTime() : timestampSupplier.getAsLong();
//               tempSensorFramePose.setToZero(sensorFrame);
//               tempSensorFramePose.changeFrame(ReferenceFrame.getWorldFrame());

               int size = ros1PointsToPublish.size();
               Point3D[] points = ros1PointsToPublish.toArray(new Point3D[size]);

               ros1PointCloudPublisher.publish(points, new float[0], "os_sensor");
            });
         }
      }
   }

   public void dispose()
   {
      if (realtimeROS2Node != null)
         realtimeROS2Node.destroy();
      depthExecutor.destroy();
      colorExecutor.destroy();
      pointCloudExecutor.destroy();
      colorROS2Executor.destroy();
      depthSensorSimulator.dispose();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderPointCloudDirectly.get())
         pointCloudRenderer.getRenderables(renderables, pool);
      if (debugCoordinateFrame.get())
      {
         if (coordinateFrame == null)
            coordinateFrame = GDXModelBuilder.createCoordinateFrameInstance(0.2);
         coordinateFrame.getRenderables(renderables, pool);
      }
      depthSensorSimulator.getVirtualRenderables(renderables, pool);
   }

   public void setSensorEnabled(boolean sensorEnabled)
   {
      this.sensorEnabled.set(sensorEnabled);
   }

   public void setRenderPointCloudDirectly(boolean renderPointCloudDirectly)
   {
      this.renderPointCloudDirectly.set(renderPointCloudDirectly);
   }

   public void setRenderDepthVideoDirectly(boolean renderDepthVideoDirectly)
   {
      getLowLevelSimulator().getDepthPanel().getIsShowing().set(renderDepthVideoDirectly);
   }

   public void setRenderColorVideoDirectly(boolean renderColorVideoDirectly)
   {
      getLowLevelSimulator().getColorPanel().getIsShowing().set(renderColorVideoDirectly);
   }

   public void setPublishDepthImageROS1(boolean publish)
   {
      publishDepthImageROS1.set(publish);
   }

   public void setPublishColorImageROS1(boolean publish)
   {
      publishColorImageROS1.set(publish);
   }

   public void setPublishPointCloudROS1(boolean publish)
   {
      publishPointCloudROS1.set(publish);
   }

   public void setPublishColorImageROS2(boolean publish)
   {
      publishColorImageROS2.set(publish);
   }

   public void setPublishPointCloudROS2(boolean publish)
   {
      publishPointCloudROS2.set(publish);
   }

   public void setDebugCoordinateFrame(boolean debugCoordinateFrame)
   {
      this.debugCoordinateFrame.set(debugCoordinateFrame);
   }

   public void setSensorFrameToWorldTransform(RigidBodyTransform sensorFrameToWorldTransform)
   {
      this.sensorFrameToWorldTransform = sensorFrameToWorldTransform;
   }

   public GDXLowLevelDepthSensorSimulator getLowLevelSimulator()
   {
      return depthSensorSimulator;
   }

   public CameraPinholeBrown getDepthCameraIntrinsics()
   {
      return depthCameraIntrinsics;
   }

   public Color getPointColorFromPicker()
   {
      return pointColorFromPicker;
   }

   public ReferenceFrame getSensorFrame()
   {
      return sensorFrame;
   }
}
