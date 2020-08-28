package us.ihmc.robotics.geometry;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameRamp3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import static us.ihmc.robotics.geometry.PlanerRegionBuilderTools.*;

public class PlanarRegionsListBuilder
{
   private PlanarRegionsList planarRegionsList = new PlanarRegionsList();
   private PoseReferenceFrame placementFrame = new PoseReferenceFrame("placementFrame", ReferenceFrame.getWorldFrame());

   public PoseReferenceFrame getPlacementFrame()
   {
      return placementFrame;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public void setRegionIds()
   {
      setRegionIds(0);
   }

   public void setRegionIds(int startId)
   {
      PlanerRegionBuilderTools.setRegionsIds(startId, planarRegionsList);
   }

   public void addBoxReferencedAtCenter(double sizeX, double sizeY, double sizeZ)
   {
      addBoxReferencedAtCenter(new Box3D(sizeX, sizeY, sizeZ));
   }

   public void addBoxReferencedAtCenter(Box3D box)
   {
      planarRegionsList.addPlanarRegionsList(createRegionsFromBox(new FrameBox3D(placementFrame, box)));
   }

   public void addRampReferencedAtCenter(Ramp3D ramp)
   {
      planarRegionsList.addPlanarRegionsList(createRegionsFromRamp(new FrameRamp3D(placementFrame, ramp)));
   }

   public void addBoxReferencedAtNegativeXYZCorner(double sizeX, double sizeY, double sizeZ)
   {
      addBoxReferencedAtNegativeXYZCorner(new Box3D(sizeX, sizeY, sizeZ));
   }

   public void addBoxReferencedAtNegativeXYZCorner(Box3D box)
   {
      placeWithOffset(new Point3D(box.getSizeX() / 2.0, box.getSizeY() / 2.0, box.getSizeZ() / 2.0), () ->
      {
         addBoxReferencedAtCenter(box);
      });
   }

   public void addBoxReferencedAtNegativeXYZCorner(Ramp3D ramp)
   {
      placeWithOffset(new Point3D(ramp.getSizeX() / 2.0, ramp.getSizeY() / 2.0, ramp.getSizeZ() / 2.0), () ->
      {
         addRampReferencedAtCenter(ramp);
      });
   }

   public void placeWithOffset(double yaw, Runnable runnable)
   {
      placeWithOffset(new AxisAngle(Axis3D.Z, yaw), runnable);
   }

   public void placeWithOffset(Orientation3DReadOnly orientationOffset, Runnable runnable)
   {
      placeWithOffset(null, orientationOffset, runnable);
   }

   public void placeWithOffset(Tuple2DReadOnly positionOffset, Runnable runnable)
   {
      placeWithOffset(new Point3D(positionOffset), runnable);
   }

   public void placeWithOffset(double x, double y, Runnable runnable)
   {
      placeWithOffset(new Point3D(x, y, 0.0), runnable);
   }

   public void placeWithOffset(double x, double y, double z, Runnable runnable)
   {
      placeWithOffset(new Point3D(x, y, z), runnable);
   }

   public void placeWithOffset(Tuple3DReadOnly positionOffset, Runnable runnable)
   {
      placeWithOffset(positionOffset, null, runnable);
   }

   public void placeWithOffset(Tuple3DReadOnly positionOffset, Orientation3DReadOnly orientationOffset, Runnable userPlacement)
   {
      Pose3D originalPlacementFrame = new Pose3D(placementFrame.getPosition(), placementFrame.getOrientation());
      Pose3D offsetPose = new Pose3D(originalPlacementFrame);
      if (positionOffset != null)
         offsetPose.getPosition().add(positionOffset);
      if (orientationOffset != null)
         offsetPose.getOrientation().append(orientationOffset);
      placementFrame.setPoseAndUpdate(offsetPose);
      userPlacement.run();
      placementFrame.setPoseAndUpdate(originalPlacementFrame);
   }
}