package us.ihmc.perception.neural;

import ai.onnxruntime.*;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.ejml.data.FMatrixRMaj;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.io.WorkspaceFile;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKeyEx;

public class HeightMapAutoencoder
{
   private static final int IMAGE_HEIGHT = 201;
   private static final int IMAGE_WIDTH = 201;

   private WorkspaceResourceDirectory modelDirectory = new WorkspaceResourceDirectory(this.getClass(), "/weights/");
   private WorkspaceFile onnxFile = new WorkspaceFile(modelDirectory, "height_map_autoencoder.onnx");

   private OrtEnvironment environment = OrtEnvironment.getEnvironment();
   private OrtSession.SessionOptions sessionOptions = new OrtSession.SessionOptions();
   private OrtSession session;

   public HeightMapAutoencoder()
   {
      try
      {
         session = environment.createSession(onnxFile.getFilesystemFile().toString(), sessionOptions);

         for (Map.Entry<String, NodeInfo> stringNodeInfoEntry : session.getInputInfo().entrySet())
         {
            LogTools.info("{}: {}", stringNodeInfoEntry.getKey(), stringNodeInfoEntry.getValue());
         }
         for (Map.Entry<String, NodeInfo> stringNodeInfoEntry : session.getOutputInfo().entrySet())
         {
            LogTools.info("{}: {}", stringNodeInfoEntry.getKey(), stringNodeInfoEntry.getValue());
         }

         //print all input names and sizes
         for (Map.Entry<String, NodeInfo> entry : session.getInputInfo().entrySet())
         {
            String inputName = entry.getKey();
            NodeInfo nodeInfo = entry.getValue();
            LogTools.info("Input Name: {}", inputName);
            LogTools.info("Input Info: {}", nodeInfo.toString());
         }
      }
      catch (OrtException e)
      {
         throw new RuntimeException(e);
      }
   }

   public Mat denoiseHeightMap(Mat heightMap, float offset)
   {
      Mat denoisedHeightMapImage = null;
      try
      {
         long startTime = System.nanoTime();

         denoisedHeightMapImage = predict(heightMap, offset);

         long endTime = System.nanoTime();
         LogTools.debug("Inference time: {} ms", (endTime - startTime) / 1000000.0);

      }
      catch (OrtException e)
      {
         throw new RuntimeException(e);
      }

      return denoisedHeightMapImage;
   }

   public Mat predict(Mat imageInput, float offset) throws OrtException
   {
      if (imageInput.rows() != IMAGE_HEIGHT || imageInput.cols() != IMAGE_WIDTH)
         throw new RuntimeException("Image height and width must be " + IMAGE_HEIGHT + " and " + IMAGE_WIDTH);

      LogTools.debug("Image Input Size: {}x{}", imageInput.rows(), imageInput.cols());

      Mat heightMapImage = imageInput.clone();

      //PerceptionDebugTools.printMat("Height Map Image", heightMapImage, 4);

      Mat heightMapInput = new Mat(IMAGE_HEIGHT, IMAGE_WIDTH, opencv_core.CV_32FC1);

      heightMapImage.convertTo(heightMapInput, opencv_core.CV_32FC1, 1, 0);
      //opencv_core.convertScaleAbs(heightMapInput, heightMapInput, 1.0 / 10000.0, 0);

      //PerceptionDebugTools.printMat("Input Image", heightMapInput, 4);

      FloatBuffer inputFloatBuffer = FloatBuffer.allocate(IMAGE_HEIGHT * IMAGE_WIDTH);

      for (int i = 0; i < IMAGE_HEIGHT; i++)
      {
         for (int j = 0; j < IMAGE_WIDTH; j++)
         {
            //LogTools.info("Reading pixel: {} {} {}", i, j, heightMapInput.ptr(i, j).getFloat());
            inputFloatBuffer.put(heightMapInput.ptr(i, j).getFloat() / 10000.0f - offset);
         }
      }
      inputFloatBuffer.rewind();

      //PerceptionDebugTools.printBuffer2D("Input Buffer", inputFloatBuffer, IMAGE_HEIGHT, IMAGE_WIDTH, 4);

      // set the image to be in the first input
      String inputName = (String) session.getInputNames().toArray()[0];
      long[] tensorInputShape = {1, 1, heightMapInput.rows(), heightMapInput.cols()};

      // create a map to store the input tensors
      Map<String, OnnxTensor> inputs = new HashMap<>();
      inputs.put(inputName, OnnxTensor.createTensor(environment, inputFloatBuffer, tensorInputShape));

      // run the inference
      OrtSession.Result output = session.run(inputs);

      Mat outputImage = new Mat(IMAGE_HEIGHT, IMAGE_WIDTH, opencv_core.CV_32FC1);
      for (Map.Entry<String, OnnxValue> stringOnnxValueEntry : output)
      {
         LogTools.debug("{}: {}", stringOnnxValueEntry.getKey(), stringOnnxValueEntry.getValue());

         OnnxTensor outputTensor = (OnnxTensor) stringOnnxValueEntry.getValue();
         float[][][][] outputArray = (float[][][][]) outputTensor.getValue();

         LogTools.debug("Output: {}", outputTensor.getInfo());

         if (outputArray[0][0].length != IMAGE_HEIGHT && outputArray[0][0][0].length != IMAGE_WIDTH)
         {
            throw new RuntimeException("Output size must be " + IMAGE_HEIGHT * IMAGE_WIDTH);
         }

         for (int i = 0; i < IMAGE_HEIGHT; i++)
         {
            for (int j = 0; j < IMAGE_WIDTH; j++)
            {
               outputImage.ptr(i, j).putFloat((outputArray[0][0][i][j] + offset) * 10000.0f);
            }
         }

         //PerceptionDebugTools.printMat("Output Image", outputImage, 4);

         // scale up the output image and convert to 16UC1
         outputImage.convertTo(outputImage, opencv_core.CV_16UC1, 1, 0);
      }

      return outputImage;
   }

   public static void main(String[] args) throws OrtException
   {
      String perceptionLogFile = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20231023_131517_PerceptionLog.hdf5").toString();

      HeightMapAutoencoder heightMapAutoencoder = new HeightMapAutoencoder();
      PerceptionDataLoader perceptionDataLoader = new PerceptionDataLoader();
      perceptionDataLoader.openLogFile(perceptionLogFile);

      BytePointer depthBytePointer = new BytePointer(1000000);
      Mat heightMap16UC1 = new Mat(201, 201, opencv_core.CV_16UC1);
      Mat outputHeightMap16UC1 = null;

      for (int i = 1; i < 100; i++)
      {
         perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.CROPPED_HEIGHT_MAP_NAME, i, depthBytePointer, heightMap16UC1);

         // Measure start time
         long startTime = System.nanoTime();

         outputHeightMap16UC1 = heightMapAutoencoder.predict(heightMap16UC1, 0.0f);

         // Measure end time
         long endTime = System.nanoTime();

         // Print time difference in ms
         LogTools.info("Inference time: {} ms", (endTime - startTime) / 1000000.0);

         // create display image for output image
         Mat outputImage16UC1 = outputHeightMap16UC1.clone();
         Mat displayOutput = new Mat(outputImage16UC1.rows(), outputImage16UC1.cols(), opencv_core.CV_8UC3);
         PerceptionDebugTools.convertDepthCopyToColor(outputImage16UC1, displayOutput);

         // create color image for input image
         Mat heightMapForDisplay = heightMap16UC1.clone();
         Mat displayInput = new Mat(heightMapForDisplay.rows(), heightMapForDisplay.cols(), opencv_core.CV_8UC3);
         PerceptionDebugTools.convertDepthCopyToColor(heightMapForDisplay, displayInput);

         // convert to color image
         Mat stackedImage = new Mat(displayInput.rows(), displayInput.cols() * 2, opencv_core.CV_8UC3);
         opencv_core.hconcat(displayInput, displayOutput, stackedImage);

         // increase brightness of the image
         opencv_core.convertScaleAbs(stackedImage, stackedImage, 4.0, 0);

         // plot and display the image using imshow, brighten the image
         opencv_imgproc.resize(stackedImage, stackedImage, new org.bytedeco.opencv.opencv_core.Size(2000, 1000));
         imshow("Display", stackedImage);
         int code = waitKeyEx(0);

         if (code == 'q')
         {
            break;
         }
      }
   }
}
