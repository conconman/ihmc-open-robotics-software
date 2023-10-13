package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

import java.util.Collection;

/**
 * This class provides support for having a reference frame that
 * doesn't always have it's designated parent available in the world
 * frame tree. So we make that designated parent it's own root and
 * place this frame under it, until the designated parent is available
 * again in the world frame tree. This is handled automatically by
 * calling the updated method.
 */
public class DetachableReferenceFrame
{
   private final RigidBodyTransform transformToParent;
   /** Never null, but does change. */
   private ReferenceFrame referenceFrame;

   public DetachableReferenceFrame(RigidBodyTransform transformToParent)
   {
      this.transformToParent = transformToParent;
   }

   public DetachableReferenceFrame()
   {
      this.transformToParent = new RigidBodyTransform();
   }

   /**
    * Note: Given frame's parent must be in the ReferenceFrameLibrary.
    */
   public void setToReferenceFrameIncludingParent(ReferenceFrame referenceFrame, Collection<ReferenceFrame> referenceFrames)
   {
      referenceFrame.getTransformToDesiredFrame(transformToParent, referenceFrame.getParent());
      update(referenceFrame.getParent().getName(), referenceFrames);
   }

   public void update(String parentFrameName, Collection<ReferenceFrame> referenceFrames)
   {
      ReferenceFrame parentFrameInWorld = null;

      for (ReferenceFrame referenceFrame : referenceFrames)
      {
         if (referenceFrame.getName().equals(parentFrameName) && ReferenceFrameMissingTools.checkIsAncestorOfWorld(referenceFrame))
         {
            parentFrameInWorld = referenceFrame;
            break;
         }
      }

      boolean shouldBeChildOfWorld = parentFrameInWorld != null;

      boolean frameNeedsRecreating = referenceFrame == null;

      if (referenceFrame != null)
      {
         frameNeedsRecreating |= shouldBeChildOfWorld != isChildOfWorld();
         frameNeedsRecreating |= referenceFrame.getParent() != parentFrameInWorld;
      }

      if (frameNeedsRecreating)
      {
         ReferenceFrame parentFrame;
         if (shouldBeChildOfWorld) // Attached to world frame tree
         {
            parentFrame = parentFrameInWorld;
         }
         else // Detached under it's own root
         {
            parentFrame = ReferenceFrameTools.constructARootFrame(parentFrameName);
         }

         referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentFrame, transformToParent);
      }
      else
      {
         referenceFrame.update(); // Neccessary?
      }
   }

   public boolean isChildOfWorld()
   {
      return ReferenceFrameMissingTools.checkIsAncestorOfWorld(referenceFrame);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }
}
