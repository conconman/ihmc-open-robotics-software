package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImBoolean;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.mesh.RDXMutableArrowModel;
import us.ihmc.rdx.mesh.RDXMutableLineModel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

public class RDXOrientationChangeDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showWorldFrame = new ImBoolean(true);
   private RDXInteractableReferenceFrame interactableStartPose;
   private RDXInteractableReferenceFrame interactableEndPose;
   private RDXMutableArrowModel rotationVectorGraphic;
   private RDXMutableLineModel rotationVectorLineGraphic;
   private RDXMutableLineModel rotationVectorLineWorldGraphic;
   private RDXReferenceFrameGraphic differenceOrientationGraphic;
   private MutableReferenceFrame endReferenceFrame = new MutableReferenceFrame();

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
            rotationVectorLineGraphic = new RDXMutableLineModel();
            rotationVectorLineWorldGraphic = new RDXMutableLineModel();

            differenceOrientationGraphic = new RDXReferenceFrameGraphic(0.32, Color.WHITE);

            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);
            baseUI.getImGuiPanelManager().addPanel("Reference Frames", this::renderImGuiWidgets);
         }

         private void renderImGuiWidgets()
         {
         }

         @Override
         public void render()
         {
            Vector3D rotationVector = new Vector3D();

            Quaternion startQuaternion = new Quaternion(interactableStartPose.getTransformToParent().getRotation());
            Quaternion endQuaternion = new Quaternion(interactableEndPose.getTransformToParent().getRotation());

            endReferenceFrame.update(rigidBodyTransform -> rigidBodyTransform.set(interactableEndPose.getTransformToParent()));

//            AxisAngle changeAxisAngle = new AxisAngle();
//            changeAxisAngle.

            FrameQuaternion differenceQuaternion = new FrameQuaternion(endReferenceFrame.getReferenceFrame());
//            differenceQuaternion.interpolate(startQuaternion, endQuaternion, 0.5);
            differenceQuaternion.difference(startQuaternion, endQuaternion);

//            differenceQuaternion.set(endQuaternion);
//            differenceQuaternion.multiplyConjugateOther(startQuaternion);

            AxisAngle differenceAxisAngle = new AxisAngle(differenceQuaternion);


            differenceQuaternion.getRotationVector(rotationVector);
            double rotationVectorMagnitude = rotationVector.norm(); // It seems we can trust the magnitude of this

            differenceQuaternion.changeFrame(ReferenceFrame.getWorldFrame());
            differenceQuaternion.getRotationVector(rotationVector);
            rotationVector.normalize();
            rotationVector.scale(rotationVectorMagnitude);

            RigidBodyTransform arrowPose = new RigidBodyTransform();
//            arrowPose.getTranslation().interpolate(interactableStartPose.getTransformToParent().getTranslation(),
//                                                   interactableEndPose.getTransformToParent().getTranslation(),
//                                                   0.5);

            arrowPose.getRotation().set(new AxisAngle(rotationVector));

            rotationVectorGraphic.update(rotationVector.norm(), Color.PINK);
            LibGDXTools.toLibGDX(arrowPose, rotationVectorGraphic.getModelInstance().transform);

            rotationVectorLineGraphic.update(new Point3D(), rotationVector, 0.01, Color.WHITE);

            FramePoint3D worldFrameVector = new FramePoint3D(endReferenceFrame.getReferenceFrame(), rotationVector);
            worldFrameVector.changeFrame(ReferenceFrame.getWorldFrame());

            Point3D vectorBase = new Point3D(interactableEndPose.getTransformToParent().getTranslation());
//            vectorBase.interpolate(interactableStartPose.getTransformToParent().getTranslation(),
//                                   interactableEndPose.getTransformToParent().getTranslation(),
//                                   0.5);
//            vectorBase.interpolate(interactableEndPose.getTransformToParent().getTranslation());
            rotationVectorLineWorldGraphic.update(vectorBase, worldFrameVector, 0.01, Color.PINK);

            differenceQuaternion.changeFrame(endReferenceFrame.getReferenceFrame());
            differenceOrientationGraphic.getFramePose3D().getOrientation().set((QuaternionReadOnly) differenceQuaternion);
            differenceOrientationGraphic.updateFromFramePose();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
         {
            if (showWorldFrame.get())
               worldFrameGraphic.getRenderables(renderables, pool);

            interactableStartPose.getVirtualRenderables(renderables, pool);
            interactableEndPose.getVirtualRenderables(renderables, pool);
//            rotationVectorGraphic.getRenderables(renderables, pool);
            rotationVectorLineGraphic.getRenderables(renderables, pool);
            rotationVectorLineWorldGraphic.getRenderables(renderables, pool);
            differenceOrientationGraphic.getRenderables(renderables, pool);
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
