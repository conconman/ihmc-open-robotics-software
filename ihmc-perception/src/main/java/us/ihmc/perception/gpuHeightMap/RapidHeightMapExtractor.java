package us.ihmc.perception.gpuHeightMap;

import org.apache.batik.ext.awt.image.renderable.PadRable;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.ArrayList;
import java.util.List;

/**
 * Extracts height map and some other cost metric maps on the GPU using OpenCL kernels
 *
 * There are two each of height map, terrain cost map, and contact map, corresponding to global and local (cropped) versions.
 * The terrain cost map is the single footstep steppability value, and the feasible contact map is the distance transform map
 * which computes the distance to closest unsteppable cell for each cell. Feasible contact map (16-bit scalar for distance transform of the terrain cost map, represents safety score
 * for distance away from boundaries and edges for each cell). For more information on Distance Transform visit:
 * https://en.wikipedia.org/wiki/Distance_transform
 * */
public class RapidHeightMapExtractor
{
   private static final double footSize = 0.22;
   private static final double distanceFromCliffTops = 0.02;
   private static final double distanceFromCliffBottoms = 0.05;
   private static final double cliffStartHeightToAvoid = 0.08;
   private static final double cliffEndHeightToAvoid = 1.2;
   private static final double minSupportAreaFraction = 0.85;
   private static final double minSnapHeightThreshold = 0.03;
   private static final double snapHeightThresholdAtSearchEdge = 0.06;
   private static final double inequalityAcvitationSlope =  50000.0;

   private int mode = 1; // 0 -> Ouster, 1 -> Realsense
   private float gridOffsetX;
   private int centerIndex;
   private int localCellsPerAxis;
   private int globalCenterIndex;
   private int cropCenterIndex;
   private int globalCellsPerAxis;
   public int sequenceNumber = 0;

   private static HeightMapParameters heightMapParameters = new HeightMapParameters("GPU");
   private final RigidBodyTransform currentSensorToWorldTransform = new RigidBodyTransform();
   private final Point3D sensorOrigin = new Point3D();

//   private HeightMapAutoencoder denoiser;
   private OpenCLManager openCLManager;
   private OpenCLFloatParameters parametersBuffer;
   private OpenCLFloatParameters snappingParametersBuffer;
   private OpenCLFloatBuffer worldToGroundTransformBuffer;
   private OpenCLFloatBuffer groundToWorldTransformBuffer;
   private OpenCLFloatBuffer groundToSensorTransformBuffer;
   private OpenCLFloatBuffer sensorToGroundTransformBuffer;
   private OpenCLFloatBuffer groundPlaneBuffer;

   private final OpenCLFloatParameters yaw = new OpenCLFloatParameters();

   private CameraIntrinsics cameraIntrinsics;
   private BytedecoImage inputDepthImage;
   private BytedecoImage localHeightMapImage;
   private BytedecoImage globalHeightMapImage;
   private BytedecoImage globalHeightVarianceImage;
   private BytedecoImage sensorCroppedHeightMapImage;
   private BytedecoImage terrainCostImage;
   private BytedecoImage contactMapImage;

   private BytedecoImage steppabilityImage;
   private BytedecoImage snapHeightImage;
   private BytedecoImage snapNormalXImage;
   private BytedecoImage snapNormalYImage;
   private BytedecoImage snapNormalZImage;

   private _cl_program rapidHeightMapUpdaterProgram;
   private _cl_program snappingHeightProgram;
   private _cl_kernel heightMapUpdateKernel;
   private _cl_kernel heightMapRegistrationKernel;
   private _cl_kernel terrainCostKernel;
   private _cl_kernel contactMapKernel;

   private _cl_kernel computeSnappedValuesKernel;

   private float[] worldToGroundTransformArray = new float[16];
   private float[] groundToWorldTransformArray = new float[16];
   private float[] groundToSensorTransformArray = new float[16];
   private float[] sensorToGroundTransformArray = new float[16];

   private Mat croppedHeightMapImage;
   private Mat denoisedHeightMap;
   private Rect cropWindowRectangle;

   private boolean modified = true;
   private boolean processing = false;
   private boolean heightMapDataAvailable = false;

   public RapidHeightMapExtractor(OpenCLManager openCLManager)
   {
      this.openCLManager = openCLManager;
//      denoiser = new HeightMapAutoencoder();
      rapidHeightMapUpdaterProgram = openCLManager.loadProgram("RapidHeightMapExtractor", "HeightMapUtils.cl");
      snappingHeightProgram = openCLManager.loadProgram("SnappingHeightMap", "HeightMapUtils.cl");
   }

   public void initialize()
   {
      recomputeDerivedParameters();
      cropWindowRectangle = new Rect((globalCellsPerAxis - heightMapParameters.getCropWindowSize()) / 2,
                                     (globalCellsPerAxis - heightMapParameters.getCropWindowSize()) / 2,
                                     heightMapParameters.getCropWindowSize(),
                                     heightMapParameters.getCropWindowSize());

      parametersBuffer = new OpenCLFloatParameters();
      snappingParametersBuffer = new OpenCLFloatParameters();
      groundToSensorTransformBuffer = new OpenCLFloatBuffer(16);
      sensorToGroundTransformBuffer = new OpenCLFloatBuffer(16);
      worldToGroundTransformBuffer = new OpenCLFloatBuffer(16);
      groundToWorldTransformBuffer = new OpenCLFloatBuffer(16);
      groundPlaneBuffer = new OpenCLFloatBuffer(4);

      groundToSensorTransformBuffer.createOpenCLBufferObject(openCLManager);
      sensorToGroundTransformBuffer.createOpenCLBufferObject(openCLManager);
      worldToGroundTransformBuffer.createOpenCLBufferObject(openCLManager);
      groundToWorldTransformBuffer.createOpenCLBufferObject(openCLManager);
      groundPlaneBuffer.createOpenCLBufferObject(openCLManager);

      croppedHeightMapImage = new Mat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);
      denoisedHeightMap = new Mat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);

      createLocalHeightMapImage(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_16UC1);
      createGlobalHeightMapImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_16UC1);
      createGlobalHeightVarianceImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      createSensorCroppedHeightMapImage(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);
      createTerrainCostImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      createContactMapImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      createSteppabilityMapImages(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);

      heightMapUpdateKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "heightMapUpdateKernel");
      heightMapRegistrationKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "heightMapRegistrationKernel");
      terrainCostKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "terrainCostKernel");
      contactMapKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "contactMapKernel");
      computeSnappedValuesKernel = openCLManager.createKernel(snappingHeightProgram, "computeSnappedValuesKernel");
   }

   public void create(BytedecoImage depthImage, int mode)
   {
      this.inputDepthImage = depthImage;
      this.mode = mode;

      initialize();
      reset();
   }

   public void recomputeDerivedParameters()
   {
      centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getLocalCellSizeInMeters());
      localCellsPerAxis = 2 * centerIndex + 1;
      gridOffsetX = (float) heightMapParameters.getLocalWidthInMeters() / 2.0f;
      globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getInternalGlobalWidthInMeters(),
                                                            heightMapParameters.getInternalGlobalCellSizeInMeters());
      globalCellsPerAxis = 2 * globalCenterIndex + 1;

      cropCenterIndex = (heightMapParameters.getCropWindowSize() - 1) / 2;

      if (2 * cropCenterIndex + 1 != heightMapParameters.getCropWindowSize())
         throw new RuntimeException("The crop center index was computed incorrectly.");
   }

   public void update(RigidBodyTransform sensorToWorldTransform, RigidBodyTransform sensorToGroundTransform, RigidBodyTransform groundToWorldTransform)
   {
      if (!processing)
      {
         currentSensorToWorldTransform.set(sensorToWorldTransform);
         sensorToGroundTransform.getTranslation().setZ(sensorToWorldTransform.getTranslationZ());

         // Upload input depth image
         inputDepthImage.writeOpenCLImage(openCLManager);

         RigidBodyTransform groundToSensorTransform = new RigidBodyTransform(sensorToGroundTransform);
         groundToSensorTransform.invert();

         RigidBodyTransform worldToGroundTransform = new RigidBodyTransform(groundToWorldTransform);
         worldToGroundTransform.invert();

         sensorOrigin.set(sensorToWorldTransform.getTranslation());

         populateParameterBuffer(heightMapParameters, cameraIntrinsics, sensorOrigin);

         // Fill world-to-sensor transform buffer
         groundToSensorTransform.get(groundToSensorTransformArray);
         groundToSensorTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(groundToSensorTransformArray);
         groundToSensorTransformBuffer.writeOpenCLBufferObject(openCLManager);

         // Fill sensor-to-world transform buffer
         sensorToGroundTransform.get(sensorToGroundTransformArray);
         sensorToGroundTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(sensorToGroundTransformArray);
         sensorToGroundTransformBuffer.writeOpenCLBufferObject(openCLManager);

         // Fill world-to-ground transform buffer
         worldToGroundTransform.get(worldToGroundTransformArray);
         worldToGroundTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(worldToGroundTransformArray);
         worldToGroundTransformBuffer.writeOpenCLBufferObject(openCLManager);

         //fill ground-to-world transform buffer
         groundToWorldTransform.get(groundToWorldTransformArray);
         groundToWorldTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(groundToWorldTransformArray);
         groundToWorldTransformBuffer.writeOpenCLBufferObject(openCLManager);

         // Set kernel arguments for the height map kernel
         openCLManager.setKernelArgument(heightMapUpdateKernel, 0, inputDepthImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 1, localHeightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 2, parametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 3, sensorToGroundTransformBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 4, groundToSensorTransformBuffer.getOpenCLBufferObject());

         // Set kernel arguments for the height map registration kernel
         openCLManager.setKernelArgument(heightMapRegistrationKernel, 0, localHeightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapRegistrationKernel, 1, globalHeightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapRegistrationKernel, 2, globalHeightVarianceImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapRegistrationKernel, 3, parametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapRegistrationKernel, 4, worldToGroundTransformBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapRegistrationKernel, 5, sensorToGroundTransformBuffer.getOpenCLBufferObject());

         // execute height map extraction and registration kernels
         openCLManager.execute2D(heightMapUpdateKernel, localCellsPerAxis, localCellsPerAxis);
         openCLManager.execute2D(heightMapRegistrationKernel, globalCellsPerAxis, globalCellsPerAxis);

         // read local and global height map images
         localHeightMapImage.readOpenCLImage(openCLManager);
         globalHeightMapImage.readOpenCLImage(openCLManager);

         // compute and read terrain cost and contact map images
         computeContactMap();
         readContactMapImage();

         // compute the steppable height image
         computeSteppabilityImage();

//         PerceptionDebugTools.printMat("Internal Original Height Map", globalHeightMapImage.getBytedecoOpenCVMat(), 600, 600, 900, 900, 10);
//         PerceptionDebugTools.printMat("Internal Snap Height Map", snapHeightImage.getBytedecoOpenCVMat(), 600, 600, 900, 900, 10);

         croppedHeightMapImage = getCroppedImage(sensorOrigin, globalCenterIndex, globalHeightMapImage.getBytedecoOpenCVMat());
         //denoisedHeightMap = denoiser.denoiseHeightMap(croppedHeightMapImage, 3.2768f);

         //PerceptionDebugTools.printMat("Cropped Height Map", croppedHeightMapImage, 4);
         //PerceptionDebugTools.printMat("Cropped Snap Height Map", croppedSnappedMapImage, 4);


         sequenceNumber++;
      }
   }

   public void populateParameterBuffer(HeightMapParameters parameters, CameraIntrinsics cameraIntrinsics, Tuple3DReadOnly gridCenter)
   {
      parametersBuffer.setParameter((float) parameters.getLocalCellSizeInMeters());
      parametersBuffer.setParameter(centerIndex);
      parametersBuffer.setParameter((float) cameraIntrinsics.getHeight());
      parametersBuffer.setParameter((float) cameraIntrinsics.getWidth());
      parametersBuffer.setParameter((float) gridCenter.getX());
      parametersBuffer.setParameter((float) gridCenter.getY());
      parametersBuffer.setParameter((float) mode);
      parametersBuffer.setParameter((float) cameraIntrinsics.getCx());
      parametersBuffer.setParameter((float) cameraIntrinsics.getCy());
      parametersBuffer.setParameter((float) cameraIntrinsics.getFx());
      parametersBuffer.setParameter((float) cameraIntrinsics.getFy());
      parametersBuffer.setParameter((float) parameters.getGlobalCellSizeInMeters());
      parametersBuffer.setParameter((float) globalCenterIndex);
      parametersBuffer.setParameter((float) parameters.getRobotCollisionCylinderRadius());
      parametersBuffer.setParameter(gridOffsetX);
      parametersBuffer.setParameter((float) parameters.getHeightFilterAlpha());
      parametersBuffer.setParameter(localCellsPerAxis);
      parametersBuffer.setParameter(globalCellsPerAxis);
      parametersBuffer.setParameter((float) parameters.getHeightScaleFactor());
      parametersBuffer.setParameter((float) parameters.getMinHeightRegistration());
      parametersBuffer.setParameter((float) parameters.getMaxHeightRegistration());
      parametersBuffer.setParameter((float) parameters.getMinHeightDifference());
      parametersBuffer.setParameter((float) parameters.getMaxHeightDifference());
      parametersBuffer.setParameter((float) parameters.getSearchWindowHeight());
      parametersBuffer.setParameter((float) parameters.getSearchWindowWidth());
      parametersBuffer.setParameter((float) cropCenterIndex);
      parametersBuffer.setParameter((float) parameters.getMinClampHeight());
      parametersBuffer.setParameter((float) parameters.getMaxClampHeight());
      parametersBuffer.setParameter((float) parameters.getHeightOffset());
      parametersBuffer.setParameter((float) parameters.getSteppingCosineThreshold());
      parametersBuffer.setParameter((float) parameters.getSteppingContactThreshold());
      parametersBuffer.setParameter((float) parameters.getContactWindowSize());
      parametersBuffer.setParameter((float) parameters.getSpatialAlpha());

      parametersBuffer.writeOpenCLBufferObject(openCLManager);


      snappingParametersBuffer.setParameter((float) gridCenter.getX());
      snappingParametersBuffer.setParameter((float) gridCenter.getY());
      snappingParametersBuffer.setParameter((float) parameters.getGlobalCellSizeInMeters());
      snappingParametersBuffer.setParameter(globalCenterIndex);
      snappingParametersBuffer.setParameter((float) cropCenterIndex);
      snappingParametersBuffer.setParameter((float) parameters.getHeightScaleFactor());
      snappingParametersBuffer.setParameter((float) parameters.getHeightOffset());
      snappingParametersBuffer.setParameter((float) footSize);
      snappingParametersBuffer.setParameter((float) footSize);
      snappingParametersBuffer.setParameter((float) distanceFromCliffTops);
      snappingParametersBuffer.setParameter((float) distanceFromCliffBottoms);
      snappingParametersBuffer.setParameter((float) cliffStartHeightToAvoid);
      snappingParametersBuffer.setParameter((float) cliffEndHeightToAvoid);
      snappingParametersBuffer.setParameter((float) minSupportAreaFraction);
      snappingParametersBuffer.setParameter((float) minSnapHeightThreshold);
      snappingParametersBuffer.setParameter((float) snapHeightThresholdAtSearchEdge);
      snappingParametersBuffer.setParameter((float) inequalityAcvitationSlope);

      snappingParametersBuffer.writeOpenCLBufferObject(openCLManager);
   }

   public void computeContactMap()
   {
      // Set kernel arguments for the terrain cost kernel
      openCLManager.setKernelArgument(terrainCostKernel, 0, globalHeightMapImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(terrainCostKernel, 1, terrainCostImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(terrainCostKernel, 2, parametersBuffer.getOpenCLBufferObject());

      // Set kernel arguments for the contact map kernel
      openCLManager.setKernelArgument(contactMapKernel, 0, terrainCostImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(contactMapKernel, 1, contactMapImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(contactMapKernel, 2, parametersBuffer.getOpenCLBufferObject());

      // Execute kernels with length and width parameters
      openCLManager.execute2D(terrainCostKernel, globalCellsPerAxis, globalCellsPerAxis);
      openCLManager.execute2D(contactMapKernel, globalCellsPerAxis, globalCellsPerAxis);
   }

   public void computeSteppabilityImage()
   {
      yaw.setParameter(0.0f); // we're only doing a single discretization, and then assuming the foot is a big rectangle
      yaw.writeOpenCLBufferObject(openCLManager);

      openCLManager.setKernelArgument(computeSnappedValuesKernel, 0, snappingParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeSnappedValuesKernel, 1, globalHeightMapImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSnappedValuesKernel, 2, yaw.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeSnappedValuesKernel, 3, steppabilityImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSnappedValuesKernel, 4, snapHeightImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSnappedValuesKernel, 5, snapNormalXImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSnappedValuesKernel, 6, snapNormalYImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSnappedValuesKernel, 7, snapNormalZImage.getOpenCLImageObject());

      openCLManager.execute2D(computeSnappedValuesKernel, heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize());

      steppabilityImage.readOpenCLImage(openCLManager);
      snapHeightImage.readOpenCLImage(openCLManager);
      snapNormalXImage.readOpenCLImage(openCLManager);
      snapNormalYImage.readOpenCLImage(openCLManager);
      snapNormalZImage.readOpenCLImage(openCLManager);
   }

   public void readContactMapImage()
   {
      // Read height map image into CPU memory
      terrainCostImage.readOpenCLImage(openCLManager);
      contactMapImage.readOpenCLImage(openCLManager);
   }

   public void reset()
   {
      localHeightMapImage.getBytedecoOpenCVMat().put(new Scalar(32768));
      localHeightMapImage.writeOpenCLImage(openCLManager);
      globalHeightMapImage.getBytedecoOpenCVMat().put(new Scalar(32768));
      globalHeightMapImage.writeOpenCLImage(openCLManager);
      snapHeightImage.getBytedecoOpenCVMat().put(new Scalar(32768));
      snapHeightImage.writeOpenCLImage(openCLManager);
      sequenceNumber = 0;
   }

   public void createLocalHeightMapImage(int height, int width, int type)
   {
      localHeightMapImage = new BytedecoImage(width, height, type);
      localHeightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void createGlobalHeightMapImage(int height, int width, int type)
   {
      globalHeightMapImage = new BytedecoImage(width, height, type);
      globalHeightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void createGlobalHeightVarianceImage(int height, int width, int type)
   {
      globalHeightVarianceImage = new BytedecoImage(width, height, type);
      globalHeightVarianceImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void createSensorCroppedHeightMapImage(int height, int width, int type)
   {
      sensorCroppedHeightMapImage = new BytedecoImage(width, height, type);
      sensorCroppedHeightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void createTerrainCostImage(int height, int width, int type)
   {
      terrainCostImage = new BytedecoImage(width, height, type);
      terrainCostImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void createContactMapImage(int height, int width, int type)
   {
      contactMapImage = new BytedecoImage(width, height, type);
      contactMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void createSteppabilityMapImages(int height, int width, int type)
   {
      steppabilityImage = new BytedecoImage(width, height, type);
      snapHeightImage = new BytedecoImage(width, height, type);
      snapNormalXImage = new BytedecoImage(width, height, type);
      snapNormalYImage = new BytedecoImage(width, height, type);
      snapNormalZImage = new BytedecoImage(width, height, type);

      steppabilityImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      snapHeightImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      snapNormalXImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      snapNormalYImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      snapNormalZImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public boolean isProcessing()
   {
      return processing;
   }

   public void setProcessing(boolean processing)
   {
      this.processing = processing;
   }

   public boolean isModified()
   {
      return modified;
   }

   public void setModified(boolean modified)
   {
      this.modified = modified;
   }

   public BytedecoImage getLocalHeightMapImage()
   {
      return localHeightMapImage;
   }

   public BytedecoImage getInternalGlobalHeightMapImage()
   {
      return globalHeightMapImage;
   }

   public BytedecoImage getSteppableHeightMapImage()
   {
      return snapHeightImage;
   }

   public BytedecoImage getSteppabilityImage()
   {
      return steppabilityImage;
   }

   public BytedecoImage getSnapNormalZImage()
   {
      return snapNormalZImage;
   }

   public Mat getCroppedGlobalHeightMapImage()
   {
      return croppedHeightMapImage;
   }

//   public Mat getDenoisedHeightMap()
//   {
//      return denoisedHeightMap;
//   }

   public Mat getSensorCroppedHeightMapImage()
   {
      return getCroppedGlobalHeightMapImage();
   }

   public Mat getCroppedTerrainCostImage()
   {
      return getCroppedImage(sensorOrigin, globalCenterIndex, terrainCostImage.getBytedecoOpenCVMat());
   }

   public Mat getCroppedContactMapImage()
   {
      return getCroppedImage(sensorOrigin, globalCenterIndex, contactMapImage.getBytedecoOpenCVMat());
   }

   public Mat getGlobalContactImage()
   {
      return contactMapImage.getBytedecoOpenCVMat();
   }

   public Mat getCroppedImage(Point3DReadOnly origin, int globalCenterIndex, Mat imageToCrop)
   {
      int xIndex = HeightMapTools.coordinateToIndex(origin.getX(), 0, RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters(), globalCenterIndex);
      int yIndex = HeightMapTools.coordinateToIndex(origin.getY(), 0, RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters(), globalCenterIndex);
      cropWindowRectangle = new Rect((yIndex - heightMapParameters.getCropWindowSize() / 2),
                                     (xIndex - heightMapParameters.getCropWindowSize() / 2),
                                     heightMapParameters.getCropWindowSize(),
                                     heightMapParameters.getCropWindowSize());
      return imageToCrop.apply(cropWindowRectangle);
   }

   public int getLocalCellsPerAxis()
   {
      return localCellsPerAxis;
   }

   public int getGlobalCellsPerAxis()
   {
      return globalCellsPerAxis;
   }

   public int getGlobalCenterIndex()
   {
      return globalCenterIndex;
   }

   public int getCenterIndex()
   {
      return centerIndex;
   }

   public Point3D getSensorOrigin()
   {
      return sensorOrigin;
   }

   public int getSequenceNumber()
   {
      return sequenceNumber;
   }

   public void setDepthIntrinsics(CameraIntrinsics cameraIntrinsics)
   {
      this.cameraIntrinsics = cameraIntrinsics;
   }

   public RigidBodyTransform getSensorToWorldTransform()
   {
      return currentSensorToWorldTransform;
   }

   public void setHeightMapDataAvailable(boolean heightMapDataAvailable)
   {
      this.heightMapDataAvailable = heightMapDataAvailable;
   }

   public boolean isHeightMapDataAvailable()
   {
      return heightMapDataAvailable;
   }

   public static HeightMapParameters getHeightMapParameters()
   {
      return heightMapParameters;
   }
}
