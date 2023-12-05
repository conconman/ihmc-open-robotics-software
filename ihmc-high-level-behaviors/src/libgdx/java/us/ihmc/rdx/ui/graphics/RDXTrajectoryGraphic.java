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

public class RDXTrajectoryGraphic
{
   private final RDXMutableLineModel positionGraphic = new RDXMutableLineModel();
   private final RDXMutableLineModel xGraphic = new RDXMutableLineModel();
   private final RDXMutableLineModel yGraphic = new RDXMutableLineModel();
   private final RDXMutableLineModel zGraphic = new RDXMutableLineModel();

   private RecyclingArrayList<RDXReferenceFrameGraphic> referenceFrameGraphics;

   private final Pose3D startPose = new Pose3D();
   private final Pose3D endPose = new Pose3D();

   private final Pose3D startOffset = new Pose3D();
   private final Pose3D endOffset = new Pose3D();
   private final RotationMatrix relativeRotation = new RotationMatrix();
   private final RotationMatrix endOrientation = new RotationMatrix();

   public void update(RigidBodyTransformReadOnly start, RigidBodyTransformReadOnly end, double lineWidth)
   {
      if (referenceFrameGraphics == null)
      {
         referenceFrameGraphics = new RecyclingArrayList<>(() -> new RDXReferenceFrameGraphic(0.03));
      }


      startPose.set(start);
      endPose.set(end);

      endOrientation.set(end.getRotation());

      relativeRotation.set(start.getRotation());
      relativeRotation.invert();
      relativeRotation.multiply(endOrientation);




      positionGraphic.update(start.getTranslation(), end.getTranslation(), lineWidth, Color.WHITE);


      referenceFrameGraphics.clear();
      for (double alpha = 0.0; alpha < 1.0; alpha += 0.1)
      {
         RDXReferenceFrameGraphic referenceFrameGraphic = referenceFrameGraphics.add();
         referenceFrameGraphic.getFramePose3D().interpolate(startPose, endPose, alpha);
         referenceFrameGraphic.updateFromFramePose();
      }


      startOffset.set(start);
      startOffset.appendTranslation(lineWidth, 0.0, 0.0);
      endOffset.set(end);
      endOffset.appendTranslation(lineWidth, 0.0, 0.0);
      xGraphic.update(startOffset.getTranslation(), endOffset.getTranslation(), lineWidth / 2.0, Color.RED);
      startOffset.set(start);
      startOffset.appendTranslation(0.0, lineWidth, 0.0);
      endOffset.set(end);
      endOffset.appendTranslation(0.0, lineWidth, 0.0);
      yGraphic.update(startOffset.getTranslation(), endOffset.getTranslation(), lineWidth / 2.0, Color.GREEN);
      startOffset.set(start);
      startOffset.appendTranslation(0.0, 0.0, lineWidth);
      endOffset.set(end);
      endOffset.appendTranslation(0.0, 0.0, lineWidth);
      zGraphic.update(startOffset.getTranslation(), endOffset.getTranslation(), lineWidth / 2.0, Color.BLUE);



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

      //      xGraphic.getRenderables(renderables, pool);
//      yGraphic.getRenderables(renderables, pool);
//      zGraphic.getRenderables(renderables, pool);
   }
}
