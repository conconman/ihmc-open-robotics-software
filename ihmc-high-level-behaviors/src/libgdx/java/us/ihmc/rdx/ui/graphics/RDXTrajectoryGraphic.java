package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.rdx.mesh.RDXMutableLineModel;

/**
 * A white line for the translation part and little coordinate frame graphics to show orientation change.
 * Orientation component is only shown if the orientation change is significant.
 */
public class RDXTrajectoryGraphic
{
   private final RDXMutableLineModel positionGraphic = new RDXMutableLineModel();

   private RecyclingArrayList<RDXReferenceFrameGraphic> referenceFrameGraphics;

   private final Pose3D startPose = new Pose3D();
   private final Pose3D endPose = new Pose3D();

   private final RotationMatrix identity = new RotationMatrix();
   private final RotationMatrix relativeRotation = new RotationMatrix();
   private final RotationMatrix endOrientation = new RotationMatrix();

   public void update(RigidBodyTransformReadOnly startInput, RigidBodyTransformReadOnly endInput, double lineWidth)
   {
      startPose.set(startInput);
      endPose.set(endInput);

      positionGraphic.update(startPose.getTranslation(), endPose.getTranslation(), lineWidth, Color.WHITE);

      handleDrawingOrientation(startInput, endInput);
   }

   private void handleDrawingOrientation(RigidBodyTransformReadOnly startInput, RigidBodyTransformReadOnly endInput)
   {
      if (referenceFrameGraphics == null)
      {
         referenceFrameGraphics = new RecyclingArrayList<>(() -> new RDXReferenceFrameGraphic(0.03));
      }
      referenceFrameGraphics.clear();

      endOrientation.set(endInput.getRotation());
      relativeRotation.set(startInput.getRotation());
      relativeRotation.invert();
      relativeRotation.multiply(endOrientation);

      double distance = relativeRotation.distance(identity);

      if (distance > 0.3) // If there's not much orientation change, don't show it.
      {
         for (double alpha = 0.0; alpha < 1.0; alpha += 0.1)
         {
            RDXReferenceFrameGraphic referenceFrameGraphic = referenceFrameGraphics.add();
            referenceFrameGraphic.getFramePose3D().interpolate(startPose, endPose, alpha);
            referenceFrameGraphic.updateFromFramePose();
         }
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      positionGraphic.getRenderables(renderables, pool);

      if (referenceFrameGraphics != null)
      {
         for (RDXReferenceFrameGraphic referenceFrameGraphic : referenceFrameGraphics)
         {
            referenceFrameGraphic.getRenderables(renderables, pool);
         }
      }
   }
}
