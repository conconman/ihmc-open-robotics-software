package us.ihmc.rdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Color;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import net.jpountz.lz4.LZ4Factory;
import net.jpountz.lz4.LZ4FastDecompressor;
import perception_msgs.msg.dds.FusedSensorHeadPointCloudMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.communication.packets.StereoPointCloudCompression;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.nio.BufferOverflowException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

/**
 * {@link ROS2PointCloudProvider} subscribes to ROS2 topic and reads in pointCloud data, stores into OpenCLFloatBuffer, and packs into {@link PointCloud}.
 * <p>
 * It expects to subscribe to {@link FusedSensorHeadPointCloudMessage}.
 * </p>
 **/

public class ROS2PointCloudProvider
{
   // Subscribe to receive message.
   private final ROS2NodeInterface ros2Node;
   private final ROS2Topic<?> topic;
   private boolean messageQueued = false;

   // Reference to message. (gets set when new message received from ros2 callback)
   private final AtomicReference<FusedSensorHeadPointCloudMessage> latestFusedSensorHeadPointCloudMessageReference = new AtomicReference<>(null);
   private final AtomicReference<LidarScanMessage> latestLidarScanMessageReference = new AtomicReference<>(null);
   private final AtomicReference<StereoVisionPointCloudMessage> latestStereoVisionMessageReference = new AtomicReference<>(null);

   // Some parameters for data processing
   private static final float POINT_SIZE = 0.01f;
   private final int pointsPerSegment;
   private final int numberOfSegments;
   private final int numberOfElementsPerPoint;
   private static final int inputBytesPerPoint = 4 * Integer.BYTES;
   private final Color color = new Color();
   private int latestSegmentIndex = -1;

   // Decompress incoming binary data (compressed)
   private final ResettableExceptionHandlingExecutorService threadQueue;
   private final LZ4FastDecompressor lz4Decompressor = LZ4Factory.nativeInstance().fastDecompressor();
   private final ByteBuffer incomingCompressedBuffer;

   // Data type PointCloud, to be updated and used globally in the future whenever sending / receiving pointCloud data.
   private final PointCloud pointCloud;

   public ROS2PointCloudProvider(ROS2NodeInterface ros2Node, ROS2Topic<?> topic, int pointsPerSegment, int numberOfSegments, int numberOfElementsPerPoint)
   {
      this.ros2Node = ros2Node;
      this.topic = topic;
      this.pointsPerSegment = pointsPerSegment;
      this.numberOfSegments = numberOfSegments;
      this.numberOfElementsPerPoint = numberOfElementsPerPoint;
      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
      incomingCompressedBuffer = ByteBuffer.allocateDirect(pointsPerSegment * numberOfSegments * inputBytesPerPoint);
      incomingCompressedBuffer.order(ByteOrder.nativeOrder());
      pointCloud = new PointCloud(pointsPerSegment * numberOfSegments, numberOfElementsPerPoint);
      subscribe();
   }

   public void create(FloatBuffer vertexBuffer)
   {

   }

   // TODO: old one, need to be deleted at some point
   private void queueRenderStereoVisionPointCloud(StereoVisionPointCloudMessage message)
   {
      //      frequencyPlot.recordEvent();
      // TODO: Possibly decompress on a thread here
      // TODO: threadQueue.clearQueueAndExecute(() ->
      latestStereoVisionMessageReference.set(message);
   }

   // TODO: old one, need to be deleted at some point
   private void queueRenderLidarScan(LidarScanMessage message)
   {
      //      frequencyPlot.recordEvent();
      latestLidarScanMessageReference.set(message);
   }

   private void queueRenderFusedSensorHeadPointCloud(FusedSensorHeadPointCloudMessage message)
   {
      messageQueued = true;
      latestFusedSensorHeadPointCloudMessageReference.set(message);
   }

   private void subscribe()
   {
      if (topic.getType().equals(LidarScanMessage.class))
      {
         new IHMCROS2Callback<>(ros2Node, topic.withType(LidarScanMessage.class), this::queueRenderLidarScan);
      }
      else if (topic.getType().equals(StereoVisionPointCloudMessage.class))
      {
         new IHMCROS2Callback<>(ros2Node, topic.withType(StereoVisionPointCloudMessage.class), this::queueRenderStereoVisionPointCloud);
      }
      else if (topic.getType().equals(FusedSensorHeadPointCloudMessage.class))
      {
         new IHMCROS2Callback<>(ros2Node,
                                topic.withType(FusedSensorHeadPointCloudMessage.class),
                                ROS2QosProfile.BEST_EFFORT(),
                                this::queueRenderFusedSensorHeadPointCloud);
      }
   }

   public int getLatestSegmentIndex()
   {
      return latestSegmentIndex;
   }

   public boolean updateFusedPointCloud()
   {
      FusedSensorHeadPointCloudMessage fusedMessage = latestFusedSensorHeadPointCloudMessageReference.getAndSet(null);
      if (fusedMessage != null)
      {
         incomingCompressedBuffer.rewind();
         int numberOfBytes = fusedMessage.getScan().size();
         incomingCompressedBuffer.limit(numberOfBytes);
         pointCloud.limit(numberOfBytes);
         for (int i = 0; i < numberOfBytes; i++)
         {
            incomingCompressedBuffer.put(fusedMessage.getScan().get(i));
         }
         incomingCompressedBuffer.flip();
         pointCloud.rewindBufferData();
         // TODO: Look at using bytedeco LZ4 1.9.X, which is supposed to be 12% faster than 1.8.X
         lz4Decompressor.decompress(incomingCompressedBuffer, pointCloud.getData());
         pointCloud.rewindBufferData();

//         pointCloud.setData(decompressedOpenCLIntBuffer.getBackingDirectByteBuffer(),
//                            pointsPerSegment * numberOfSegments * pointCloud.getNumberOfElementsPerPoint());

         latestSegmentIndex = (int) fusedMessage.getSegmentIndex();

//         pointCloud.setData(discretizedPointBuffer.getBytedecoFloatBufferPointer(),
//                            pointsPerSegment * numberOfSegments * pointCloud.getNumberOfElementsPerPoint());

         return true;
      }

      return false;
   }

   public Function<FloatBuffer, Integer> updateAndGetBufferConsumer()
   {
      LidarScanMessage latestLidarScanMessage = latestLidarScanMessageReference.getAndSet(null);
      if (latestLidarScanMessage != null)
      {
         int numberOfScanPoints = latestLidarScanMessage.getNumberOfPoints();

         return xyzRGBASizeFloatBuffer ->
         {
            LidarPointCloudCompression.decompressPointCloud(latestLidarScanMessage.getScan(), numberOfScanPoints, (i, x, y, z) ->
            {
               xyzRGBASizeFloatBuffer.put((float) x);
               xyzRGBASizeFloatBuffer.put((float) y);
               xyzRGBASizeFloatBuffer.put((float) z);
               xyzRGBASizeFloatBuffer.put(color.r);
               xyzRGBASizeFloatBuffer.put(color.g);
               xyzRGBASizeFloatBuffer.put(color.b);
               xyzRGBASizeFloatBuffer.put(color.a);
            });

            return numberOfScanPoints;
         };
      }

      StereoVisionPointCloudMessage latestStereoVisionMessage = latestStereoVisionMessageReference.getAndSet(null);
      if (latestStereoVisionMessage != null)
      {
         return xyzRGBASizeFloatBuffer ->
         {
            StereoPointCloudCompression.decompressPointCloud(latestStereoVisionMessage, (x, y, z) ->
            {
               try
               {
                  xyzRGBASizeFloatBuffer.put((float) x);
                  xyzRGBASizeFloatBuffer.put((float) y);
                  xyzRGBASizeFloatBuffer.put((float) z);
                  xyzRGBASizeFloatBuffer.put(color.r);
                  xyzRGBASizeFloatBuffer.put(color.g);
                  xyzRGBASizeFloatBuffer.put(color.b);
                  xyzRGBASizeFloatBuffer.put(color.a);
               }
               catch (BufferOverflowException e)
               {
                  e.printStackTrace();
               }
            });

            return latestStereoVisionMessage.getNumberOfPoints();
         };
      }
      return xyzRGBASizeFloatBuffer -> 0;
   }

   public PointCloud getPointCloud()
   {
      return pointCloud;
   }

   public boolean pollMessageQueued()
   {
      boolean ret = messageQueued;
      messageQueued = false;
      return ret;
   }
}
