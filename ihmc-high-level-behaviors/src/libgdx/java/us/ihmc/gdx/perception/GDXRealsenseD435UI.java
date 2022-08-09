package us.ihmc.gdx.perception;

import imgui.ImGui;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.time.FrequencyCalculator;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.nio.ByteOrder;

public class GDXRealsenseD435UI
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/main/resources");
   private final Activator nativesLoadedActivator;
   private final GDXPose3DGizmo sensorPoseGizmo = new GDXPose3DGizmo();
   private YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense d435;
   private GDXCVImagePanel depthImagePanel;
   private Mat depthU16C1Image;
   private BytedecoImage depth32FC1Image;
   private FrequencyCalculator frameReadFrequency = new FrequencyCalculator();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public GDXRealsenseD435UI()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            ImGuiPanel panel = new ImGuiPanel("D435", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  realSenseHardwareManager = new RealSenseHardwareManager(yoRegistry, yoGraphicsListRegistry);

                  d435 = realSenseHardwareManager.createD435("049222073352", 1280, 720, 30);
//                  d435.enableColor(1920, 1080, 30);
                  d435.initialize();
               }

               if (d435.readFrameData())
               {
                  d435.updateDataBytePointers();

                  if (depthImagePanel == null)
                  {
                     MutableBytePointer depthFrameData = d435.getDepthFrameData();
                     depthU16C1Image = new Mat(d435.getDepthHeight(), d435.getDepthWidth(), opencv_core.CV_16UC1, depthFrameData);

                     depth32FC1Image = new BytedecoImage(d435.getDepthWidth(), d435.getDepthHeight(), opencv_core.CV_32FC1);
                     depthImagePanel = new GDXCVImagePanel("D435 Depth", d435.getDepthWidth(), d435.getDepthHeight());
                     baseUI.getImGuiPanelManager().addPanel(depthImagePanel.getVideoPanel());

                     baseUI.getPerspectiveManager().reloadPerspective();
                  }

                  frameReadFrequency.ping();
                  depthU16C1Image.convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, d435.getDepthToMeterConversion(), 0.0);

                  depthImagePanel.drawFloatImage(depth32FC1Image.getBytedecoOpenCVMat());
               }
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());

            if (depthImagePanel != null)
            {
               ImGui.text("Depth frame data size: " + d435.getDepthFrameDataSize());
               ImGui.text("Frame read frequency: " + frameReadFrequency.getFrequency());
               ImGui.text("Depth to meters conversion: " + d435.getDepthToMeterConversion());

               ImGui.text("Unsigned 16 Depth:");

               for (int i = 0; i < 5; i++)
               {
                  ImGui.text(depthU16C1Image.ptr(0, i).getShort() + " ");
               }

               ImGui.text("Float 32 Meters:");

               depth32FC1Image.rewind();
               for (int i = 0; i < 5; i++)
               {
                  ImGui.text(depth32FC1Image.getBackingDirectByteBuffer().getFloat() + " ");
               }

               ImGui.text("R G B A:");

               depthImagePanel.getBytedecoImage().rewind();
               for (int i = 0; i < 5; i++)
               {
                  printBytes(depthImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(0),
                             depthImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(1),
                             depthImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(2),
                             depthImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(3));
               }
            }
         }

         private void printBytes(byte byte0, byte byte1, byte byte2, byte byte3)
         {
            printInts(Byte.toUnsignedInt(byte0),
                      Byte.toUnsignedInt(byte1),
                      Byte.toUnsignedInt(byte2),
                      Byte.toUnsignedInt(byte3));
         }

         private void printInts(int int0, int int1, int int2, int int3)
         {
            ImGui.text(int0 + " " + int1 + " " + int2 + " " + int3);
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            d435.deleteDevice();
            realSenseHardwareManager.deleteContext();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXRealsenseD435UI();
   }
}
