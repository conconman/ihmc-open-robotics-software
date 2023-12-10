package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImBoolean;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.mesh.RDXMutableArrowModel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;

public class RDXOrientationChangeDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showWorldFrame = new ImBoolean(true);
   private RDXInteractableReferenceFrame interactableStartPose;
   private RDXInteractableReferenceFrame interactableEndPose;
   private RDXMutableArrowModel rotationVectorGraphic;

   public RDXOrientationChangeDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         private ModelInstance worldFrameGraphic;

         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimary3DPanel().getCamera3D().setCameraFocusPoint(new Point3D(0.7, 0.0, 0.4));
            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(-3.0, -4.0, 4.0);

            worldFrameGraphic = new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3));
            LibGDXTools.setOpacity(worldFrameGraphic, 0.6f);

            interactableStartPose = new RDXInteractableReferenceFrame();
            interactableStartPose.createAndSetupDefault(baseUI, ReferenceFrame.getWorldFrame(), 0.2);
            interactableStartPose.getTransformToParent().getTranslation().set(0.5, 0.2, 1.0);
            interactableStartPose.getRepresentativeReferenceFrame().update();

            interactableEndPose = new RDXInteractableReferenceFrame();
            interactableEndPose.createAndSetupDefault(baseUI, ReferenceFrame.getWorldFrame(), 0.2);
            interactableEndPose.getTransformToParent().getTranslation().set(0.5, -0.2, 1.0);
            interactableEndPose.getRepresentativeReferenceFrame().update();

            rotationVectorGraphic = new RDXMutableArrowModel();

            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);
            baseUI.getImGuiPanelManager().addPanel("Reference Frames", this::renderImGuiWidgets);
         }

         private void renderImGuiWidgets()
         {
         }

         @Override
         public void render()
         {
            Quaternion startQuaternion = new Quaternion(interactableStartPose.getTransformToParent().getRotation());
            Quaternion endQuaternion = new Quaternion(interactableEndPose.getTransformToParent().getRotation());

            Quaternion differenceQuaternion = new Quaternion();
            differenceQuaternion.interpolate(startQuaternion, endQuaternion, 0.5);
//            differenceQuaternion.difference(startQuaternion, endQuaternion);

            Vector3D rotationVector = new Vector3D();
            differenceQuaternion.getRotationVector(rotationVector);


            RigidBodyTransform arrowPose = new RigidBodyTransform();
            arrowPose.getTranslation().interpolate(interactableStartPose.getTransformToParent().getTranslation(),
                                                   interactableEndPose.getTransformToParent().getTranslation(),
                                                   0.5);

            arrowPose.getRotation().set(differenceQuaternion);


            rotationVectorGraphic.update(rotationVector.norm(), Color.PINK);
            LibGDXTools.toLibGDX(arrowPose, rotationVectorGraphic.getModelInstance().transform);

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
         {
            if (showWorldFrame.get())
               worldFrameGraphic.getRenderables(renderables, pool);

            interactableStartPose.getVirtualRenderables(renderables, pool);
            interactableEndPose.getVirtualRenderables(renderables, pool);
            rotationVectorGraphic.getRenderables(renderables, pool);
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXOrientationChangeDemo();
   }
}
