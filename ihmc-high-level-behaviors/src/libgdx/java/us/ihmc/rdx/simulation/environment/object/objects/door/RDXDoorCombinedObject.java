package us.ihmc.rdx.simulation.environment.object.objects.door;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.simulation.bullet.RDXBulletPhysicsManager;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;

public class RDXDoorCombinedObject extends RDXEnvironmentObject
{
   public static final String NAME = "Door Combined";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXDoorCombinedObject.class);
   private final RDXDoorFrameObject doorFrameObject;
   private final RDXDoorPanelObject doorPanelObject;
   private final RDXDoorLeverHandleObject doorLeverObject;

   public RDXDoorCombinedObject()
   {
      super(NAME, FACTORY);

      doorFrameObject = new RDXDoorFrameObject();
      doorPanelObject = new RDXDoorPanelObject();
      doorLeverObject = new RDXDoorLeverHandleObject();
   }

   @Override
   public void addToBullet(RDXBulletPhysicsManager bulletPhysicsManager)
   {
      doorFrameObject.addToBullet(bulletPhysicsManager);
      doorPanelObject.addToBullet(bulletPhysicsManager);
      doorLeverObject.addToBullet(bulletPhysicsManager);
   }

   @Override
   public void removeFromBullet()
   {
      super.removeFromBullet();
      doorFrameObject.removeFromBullet();
      doorPanelObject.removeFromBullet();
      doorLeverObject.removeFromBullet();
   }

   @Override
   public void updateRenderablesPoses()
   {
      // do nothing
   }

   @Override
   public boolean intersect(Line3DReadOnly pickRay, Point3D intersectionToPack)
   {
      return doorFrameObject.intersect(pickRay, intersectionToPack);
   }

   @Override
   public void setSelected(boolean selected)
   {
      doorFrameObject.setSelected(selected);
   }

   @Override
   public void setPositionInWorld(Point3DReadOnly positionInWorld)
   {
      doorFrameObject.setPositionInWorld(positionInWorld);
   }

   @Override
   public void setPoseInWorld(Pose3D poseInWorld)
   {
      doorFrameObject.setPoseInWorld(poseInWorld);
   }

   @Override
   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      doorFrameObject.setTransformToWorld(transformToWorld);
   }

   @Override
   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      doorFrameObject.getRealRenderables(renderables, pool);
      doorPanelObject.getRealRenderables(renderables, pool);
      doorLeverObject.getRealRenderables(renderables, pool);
   }

   @Override
   public void getCollisionMeshRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      doorFrameObject.getCollisionMeshRenderables(renderables, pool);
      doorPanelObject.getCollisionMeshRenderables(renderables, pool);
      doorLeverObject.getCollisionMeshRenderables(renderables, pool);
   }

   @Override
   public RigidBodyTransform getObjectTransform()
   {
      return doorFrameObject.getObjectTransform();
   }

//   public static final renderImGuiWidgets()
//   {
//      ImGui.sliderFloat("Low limit", )
//   }
}
