package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.ihmcPerception.depthData.PointCloudData;
import us.ihmc.ihmcPerception.heightMap.HeightMapInputData;
import us.ihmc.ihmcPerception.heightMap.HeightMapUpdater;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.tools.PerceptionDataTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXHeightMapRenderer;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.ros2.RDXHeightMapVisualizer;
import us.ihmc.rdx.visualizers.RDXHeightMapGraphic;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.nio.FloatBuffer;
import java.time.Instant;
import java.util.ArrayList;

public class RDXHeightMapExtractionDemo
{
   private final String perceptionLogFile = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20230117_161540_GoodPerceptionLog.hdf5").toString();

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private ImGuiPanel navigationPanel;

   private String sensorTopicName;

   private final ArrayList<Point3D> sensorPositionBuffer = new ArrayList<>();
   private final ArrayList<Quaternion> sensorOrientationBuffer = new ArrayList<>();

   private final HeightMapUpdater heightMapUpdater = new HeightMapUpdater();
   private final RDXHeightMapVisualizer heightMapVisualizer = new RDXHeightMapVisualizer();

   private final Notification userChangedIndex = new Notification();
   private final ResettableExceptionHandlingExecutorService loadAndDecompressThreadExecutor = MissingThreadTools.newSingleThreadExecutor("LoadAndDecompress",
                                                                                                                                         true,
                                                                                                                                         1);

   private ImInt frameIndex = new ImInt(0);
   private ImFloat planeHeight = new ImFloat(1.5f); // 2.133f

   private final Pose3D cameraPose = new Pose3D();
   private final PoseReferenceFrame cameraFrame = new PoseReferenceFrame("l515ReferenceFrame", ReferenceFrame.getWorldFrame());

   private final Notification heightMapUpdateNotification = new Notification();

   private Activator nativesLoadedActivator;

   private BytedecoImage loadedDepthImage;
   private final BytePointer depthBytePointer = new BytePointer(1000000);

   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private PerceptionDataLoader perceptionDataLoader;

   private boolean initialized = false;

   public RDXHeightMapExtractionDemo()
   {
      perceptionDataLoader = new PerceptionDataLoader();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();
            baseUI.create();

            openCLManager = new OpenCLManager();
            openCLProgram = openCLManager.loadProgram("RapidHeightMapExtractor");

            heightMapVisualizer.create();
            heightMapVisualizer.setActive(true);
            heightMapUpdater.attachHeightMapConsumer(heightMapVisualizer::acceptHeightMapMessage);

            navigationPanel = new ImGuiPanel("Dataset Navigation Panel");
            baseUI.getImGuiPanelManager().addPanel(navigationPanel);

            baseUI.getPrimaryScene().addRenderableProvider(heightMapVisualizer);

            createForOuster(128, 2048);

            updateHeightMap();

            // testProjection(loadedDepthImage.getBytedecoOpenCVMat());
         }

         private void createForOuster(int depthHeight, int depthWidth)
         {
            sensorTopicName = PerceptionLoggerConstants.OUSTER_DEPTH_NAME;
            perceptionDataLoader.openLogFile(perceptionLogFile);

            loadedDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);

            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.OUSTER_SENSOR_POSITION, sensorPositionBuffer);
            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.OUSTER_SENSOR_ORIENTATION, sensorOrientationBuffer);

            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.OUSTER_DEPTH_NAME,
                                                     frameIndex.get(),
                                                     depthBytePointer,
                                                     loadedDepthImage.getBytedecoOpenCVMat());
            loadedDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);

         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  baseUI.getLayoutManager().reloadLayout();
                  navigationPanel.setRenderMethod(this::renderNavigationPanel);
               }

               if (userChangedIndex.poll())
               {
                  loadAndDecompressThreadExecutor.clearQueueAndExecute(() ->
                                                                       {
                                                                          perceptionDataLoader.loadCompressedDepth(sensorTopicName,
                                                                                                                   frameIndex.get(),
                                                                                                                   depthBytePointer,
                                                                                                                   loadedDepthImage.getBytedecoOpenCVMat());
                                                                       });
                  updateHeightMap();
               }
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderNavigationPanel()
         {
            boolean changed = ImGui.sliderInt("Frame Index",
                                              frameIndex.getData(),
                                              0,
                                              (int) (perceptionDataLoader.getHDF5Manager().getCount(sensorTopicName) - 1));

            changed |= ImGui.sliderFloat("Plane Height", planeHeight.getData(), -3.0f, 3.0f);

            if (ImGui.button("Load Previous"))
            {
               frameIndex.set(Math.max(0, frameIndex.get() - 1));
               changed = true;
            }
            ImGui.sameLine();
            if (ImGui.button("Load Next"))
            {
               frameIndex.set(frameIndex.get() + 1);
               changed = true;
            }

            if (changed)
            {
               userChangedIndex.set();
            }
         }

         @Override
         public void dispose()
         {
            perceptionDataLoader.closeLogFile();
            openCLManager.destroy();
            baseUI.dispose();
         }
      });
   }

   public void updateWithDataBuffer(ReferenceFrame groundFrame,
                                    ReferenceFrame sensorFrame,
                                    BytedecoImage depthImage,
                                    int numberOfPoints,
                                    Instant instant)
   {

      FramePose3D sensorPose = new FramePose3D(sensorFrame);
      sensorPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D gridCenter = new Point3D(sensorPose.getX(), sensorPose.getY(), 0.0);
      FloatBuffer pointCloudInSensorFrame = PerceptionDataTools.convertPolarDepthImageToPointCloudInSensorFrame(depthImage, Math.PI / 2.0, 2.0 * Math.PI);
      PointCloudData pointCloudData = new PointCloudData(instant, numberOfPoints, pointCloudInSensorFrame);
      HeightMapInputData inputData = new HeightMapInputData();
      inputData.pointCloud = pointCloudData;
      inputData.gridCenter = gridCenter;
      // submitting the world frame for the sensor pose, as that's the frame the data is in.
      inputData.sensorPose = sensorPose;
      // TODO add variance
//      if (currentWalkingStatus.get() == WalkingStatus.STARTED)
//      {
         inputData.verticalMeasurementVariance = heightMapUpdater.getHeightMapParameters().getSensorVarianceWhenMoving();
//      }
//      else
//      {
         inputData.verticalMeasurementVariance = heightMapUpdater.getHeightMapParameters().getSensorVarianceWhenStanding();
//      }

      heightMapUpdater.addPointCloudToQueue(inputData);
   }

   private void updateHeightMap()
   {

         LogTools.info("Update Height Map: " + frameIndex.get());
         Point3D position = sensorPositionBuffer.get(frameIndex.get());
         Quaternion orientation = sensorOrientationBuffer.get(frameIndex.get());
         cameraPose.set(position, orientation);
         cameraFrame.setPoseAndUpdate(cameraPose);

         updateWithDataBuffer(ReferenceFrame.getWorldFrame(),
                              cameraFrame,
                              loadedDepthImage,
                              loadedDepthImage.getImageHeight() * loadedDepthImage.getImageWidth(),
                              Instant.now());
         heightMapUpdater.runUpdateThread();

         heightMapVisualizer.update();

//      if (heightMapUpdateNotification.poll())
//      {
//         heightMapRenderer.update(rapidHeightMapUpdater.getOutputHeightMapImage().getPointerForAccessSpeed(),
//                                  rapidHeightMapUpdater.getGridLength(),
//                                  rapidHeightMapUpdater.getGridWidth(),
//                                  rapidHeightMapUpdater.getCellSizeXYInMeters());
//
//         rapidHeightMapUpdater.setModified(false);
//         rapidHeightMapUpdater.setProcessing(false);
//
//         PerceptionDebugTools.displayHeightMap("Output Height Map",
//                                               rapidHeightMapUpdater.getOutputHeightMapImage().getBytedecoOpenCVMat(),
//                                               1,
//                                              1 / (0.3f + 0.20f * rapidHeightMapUpdater.getCellSizeXYInMeters()));
//      }
   }




   public static void main(String[] args)
   {
      new RDXHeightMapExtractionDemo();
   }
}
