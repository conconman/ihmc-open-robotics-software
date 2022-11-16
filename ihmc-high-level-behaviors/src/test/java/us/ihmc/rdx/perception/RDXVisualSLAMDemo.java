package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
//import us.ihmc.log.LogTools;
import us.ihmc.perception.ImageMat;
import us.ihmc.perception.ImageTools;
import us.ihmc.perception.VisualSLAMModule;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.util.ArrayList;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class RDXVisualSLAMDemo
{
   private final ScheduledExecutorService executor = ExecutorServiceTools.newSingleThreadScheduledExecutor(getClass(),
                                                                                                           ExecutorServiceTools.ExceptionHandling.CANCEL_AND_REPORT);

   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private final VisualSLAMModule vslam;
   private final ImGuiPanel panel = new ImGuiPanel("Visual SLAM");

   private static final int COUNT_SKIP = 100;
   private static final int FRAME_SKIP = 1;
   private static final String LEFT_CAMERA_NAME = "image_0";
   private static final String RIGHT_CAMERA_NAME = "image_1";
   private static final String DATASET_PATH = "/home/quantum/Workspace/Data/Datasets/sequences/00/";

   private ImageMat currentImageRight;
   private ImageMat currentImageLeft;

   private String leftImageName;
   private String rightImageName;
   private String fileName = "000000.png";

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ArrayList<ModelInstance> poseModels = new ArrayList<>();

   private ModelInstance modelInstance;
   private ModelInstance landmarksLineMesh;

   private ReferenceFrame frame;

   private boolean active = false;
   private int updateCount = 0;
   private int fileIndex = 0;

   public RDXVisualSLAMDemo()
   {
      vslam = new VisualSLAMModule();
      panel.setRenderMethod(this::renderImGuiWidgets);

      baseUI.getImGuiPanelManager().addPanel(panel);
      baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {

            baseUI.create();

            //ModelBuilder modelBuilder = new ModelBuilder();
            //modelBuilder.begin();
            //
            //RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
            //meshBuilder.addLine(0,0,0, 1, 1, 1, 0.005, Color.WHITE);
            //Mesh mesh = meshBuilder.generateMesh();
            //
            //MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
            //Material material = new Material();
            //Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
            //material.set(TextureAttribute.createDiffuse(paletteTexture));
            //material.set(ColorAttribute.createDiffuse(Color.WHITE));
            //modelBuilder.part(meshPart, material);
            //
            //Model model = modelBuilder.end();
            //landmarksLineMesh = new ModelInstance(model);

         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();

            if(active)
            {
               //if(updateCount % COUNT_SKIP == 0)
               {
                  update();
               }
               updateCount++;
            }

            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            executor.shutdown();
            baseUI.dispose();
         }
      });
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.button("Next"))
      {
         update();
      }

      if (ImGui.button("Start"))
      {
         active = true;
         //executor.scheduleAtFixedRate(this::update, 0, 20L, TimeUnit.MILLISECONDS);
      }

      if (ImGui.button("Pause"))
      {
         active = false;
      }
   }

   public void update()
   {
      //LogTools.info("Loading File Index: {}", fileIndex);
      fileName = String.format("%1$6s", fileIndex).replace(' ', '0') + ".png";
      leftImageName = DATASET_PATH + LEFT_CAMERA_NAME + "/" + fileName;
      rightImageName = DATASET_PATH + RIGHT_CAMERA_NAME + "/" + fileName;

      currentImageLeft = ImageTools.loadAsImageMat(leftImageName);
      currentImageRight = ImageTools.loadAsImageMat(rightImageName);

      boolean initialized = vslam.update(currentImageLeft, currentImageRight);

      fileIndex += FRAME_SKIP;

      if(initialized)
      {
         /* For Visualization Only */
         poseModels.clear();
         for (int i = 0; i < fileIndex/FRAME_SKIP; i++)
         {
            FramePose3D framePose = vslam.getSensorPose(i);
            framePose.changeFrame(ReferenceFrame.getWorldFrame());

            ////LogTools.info("Optimized Sensor Pose: \n{}\n", framePose);
            modelInstance = RDXModelBuilder.createCoordinateFrameInstance(0.4);
            LibGDXTools.toLibGDX(framePose, tempTransform, modelInstance.transform);
            modelInstance.transform.val[Matrix4.M03] *= 0.1;
            modelInstance.transform.val[Matrix4.M13] *= 0.1;
            modelInstance.transform.val[Matrix4.M23] *= 0.1;
            poseModels.add(modelInstance);
         }

         //vslam.clearISAM2();
      }
      //LogTools.info("Total Model Instances: {}", poseModels.size());
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ModelInstance model : poseModels)
      {
         model.getRenderables(renderables, pool);
      }
      //landmarksLineMesh.getRenderables(renderables, pool);
   }

   public static void main(String[] args)
   {
      new RDXVisualSLAMDemo();
   }
}
