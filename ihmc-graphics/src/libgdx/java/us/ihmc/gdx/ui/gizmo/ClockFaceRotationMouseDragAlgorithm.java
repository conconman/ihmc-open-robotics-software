package us.ihmc.gdx.ui.gizmo;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class ClockFaceRotationMouseDragAlgorithm
{
   private final Line3D axisLineAtObjectZero = new Line3D();
   private final Plane3D pickClockPlane = new Plane3D();
   private final Point3D clockCenter = new Point3D();
   private final Point3D clockHandTipInWorld = new Point3D();
   private final Point3D previousClockHandTipInWorld = new Point3D();
   private final Vector3D clockHand = new Vector3D();
   private final Vector3D previousClockHand = new Vector3D();
   private final Vector3D crossProduct = new Vector3D();
   private final AxisAngle axisAngleToRotateBy = new AxisAngle();

   public void reset()
   {
      previousClockHandTipInWorld.setToNaN();
   }

   public boolean calculate(Line3DReadOnly pickRay,
                         Point3D objectRayCollision,
                         RotationMatrix axisOrientationInObjectFrame,
                         RigidBodyTransformReadOnly objectTransform)
   {
      axisLineAtObjectZero.getPoint().set(objectTransform.getTranslation());
      axisLineAtObjectZero.getDirection().set(Axis3D.Z);
      axisOrientationInObjectFrame.transform(axisLineAtObjectZero.getDirection());
      objectTransform.getRotation().transform(axisLineAtObjectZero.getDirection());

      boolean hasMotion = false;
      if (previousClockHandTipInWorld.containsNaN())
      {
         pickClockPlane.set(objectRayCollision, axisLineAtObjectZero.getDirection());
         pickClockPlane.intersectionWith(axisLineAtObjectZero, clockCenter);
         pickClockPlane.getPoint().set(clockCenter);

         previousClockHandTipInWorld.set(objectRayCollision);
      }
      else
      {
         pickClockPlane.intersectionWith(clockHandTipInWorld, pickRay.getPoint(), pickRay.getDirection());

         clockHand.set(clockHandTipInWorld.getX() - pickClockPlane.getPoint().getX(),
                       clockHandTipInWorld.getY() - pickClockPlane.getPoint().getY(),
                       clockHandTipInWorld.getZ() - pickClockPlane.getPoint().getZ());
         previousClockHand.set(previousClockHandTipInWorld.getX() - pickClockPlane.getPoint().getX(),
                               previousClockHandTipInWorld.getY() - pickClockPlane.getPoint().getY(),
                               previousClockHandTipInWorld.getZ() - pickClockPlane.getPoint().getZ());

         double angleFromPreviousToCurrentHand = EuclidGeometryTools.angleFromFirstToSecondVector3D(previousClockHand.getX(),
                                                                                                    previousClockHand.getY(),
                                                                                                    previousClockHand.getZ(),
                                                                                                    clockHand.getX(),
                                                                                                    clockHand.getY(),
                                                                                                    clockHand.getZ());

         if (hasMotion = !Double.isNaN(angleFromPreviousToCurrentHand))
         {
            crossProduct.cross(previousClockHand, clockHand);
            if (crossProduct.dot(pickClockPlane.getNormal()) < 0.0)
               angleFromPreviousToCurrentHand = -angleFromPreviousToCurrentHand;

            axisAngleToRotateBy.set(pickClockPlane.getNormal(), angleFromPreviousToCurrentHand);
         }

         previousClockHandTipInWorld.set(clockHandTipInWorld);
      }
      return hasMotion;
   }

   public AxisAngle getMotion()
   {
      return axisAngleToRotateBy;
   }
}
