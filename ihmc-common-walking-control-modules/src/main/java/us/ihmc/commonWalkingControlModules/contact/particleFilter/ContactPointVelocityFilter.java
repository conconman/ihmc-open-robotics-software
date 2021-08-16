package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tools.EuclidCoreTools;

import java.util.function.Predicate;

public class ContactPointVelocityFilter implements Predicate<ContactPointParticle>
{
   private static final double defaultMinSpeedToReject = 0.01;
   private static final double defaultAngleThreshold = Math.toRadians(20.0);

   /* Only particles with speeds above this are considered for rejection */
   private double minSpeedToReject;
   /* If zero angle is tangent to the mesh and pi is into the mesh, this is the angular cutoff above which points are filtered */
   private double angleThreshold;

   public ContactPointVelocityFilter()
   {
      reset();
   }

   public void reset()
   {
      minSpeedToReject = defaultMinSpeedToReject;
      angleThreshold = defaultAngleThreshold;
   }

   public void setMinSpeedToReject(double minSpeedToReject)
   {
      this.minSpeedToReject = minSpeedToReject;
   }

   public void setAngleThreshold(double angleThreshold)
   {
      this.angleThreshold = angleThreshold;
   }

   @Override
   public boolean test(ContactPointParticle contactPointParticle)
   {
      FrameVector3D contactPointVelocity = contactPointParticle.getContactPointVelocity();
      contactPointVelocity.changeFrame(contactPointParticle.getContactPointFrame());

      // quick check if the velocity has positive z it isn't filtered
      if (contactPointVelocity.getZ() > 0.0)
      {
         return false;
      }

      if (contactPointVelocity.lengthSquared() < MathTools.square(minSpeedToReject))
      {
         return false;
      }

      double angleRelativeToTangent = Math.asin(Math.abs(contactPointVelocity.getZ()) / EuclidCoreTools.fastNorm(contactPointVelocity.getX(), contactPointVelocity.getY()));
      return angleRelativeToTangent > angleThreshold;
   }
}
