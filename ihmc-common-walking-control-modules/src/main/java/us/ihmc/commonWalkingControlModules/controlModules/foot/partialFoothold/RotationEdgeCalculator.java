package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;

public interface RotationEdgeCalculator
{
   void compute(FramePoint2DReadOnly measuredCoP);

   void reset();

   FrameLine2DReadOnly getLineOfRotation();

   static double getLineVizWidth()
   {
      return 0.1;
   }
}
