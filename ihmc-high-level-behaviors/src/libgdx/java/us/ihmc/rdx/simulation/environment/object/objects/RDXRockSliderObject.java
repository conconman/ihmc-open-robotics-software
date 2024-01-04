package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btConstraintSetting;
import com.badlogic.gdx.physics.bullet.dynamics.btPoint2PointConstraint;
import com.badlogic.gdx.physics.bullet.dynamics.btSliderConstraint;
import com.badlogic.gdx.physics.bullet.linearmath.btTransform;
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

public class RDXRockSliderObject extends RDXEnvironmentObject
{
   public static final String NAME = "Rock Slider";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXMultiBodySnakeObject.class);
   private final RDXRockObject rockParentObject;
   private final RDXRockObject rockChildObject;
   private final RDXRockObject rockSliderObject;

   public RDXRockSliderObject()
   {
      super(NAME, FACTORY);

      rockParentObject = new RDXRockObject(40, 0.5, 0.3, 0f, 4215);
      rockChildObject = new RDXRockObject(31, 0.5, 0.3, 60f, 487);
      rockSliderObject = new RDXRockObject(31, 0.5, 0.3, 150f, 521);
   }

   @Override
   public void addToBullet(RDXBulletPhysicsManager bulletPhysicsManager)
   {
      rockParentObject.addToBullet(bulletPhysicsManager);
      rockChildObject.addToBullet(bulletPhysicsManager);
      rockSliderObject.addToBullet(bulletPhysicsManager);
      
      Matrix4 frameInA, frameInB;
      frameInA = btTransform.getIdentity();
      frameInB = btTransform.getIdentity();
      
      // the slider constraint is x aligned per default, but we want it to be y aligned, therefore we rotate it
      frameInA.setFromEulerAnglesRad(0f, 1.5f, (float)-Math.PI / 2.0f);  //we use Y like up Axis
      frameInB.setFromEulerAnglesRad(0f, 1.5f, (float)-Math.PI / 2.0f);  //we use Y like up Axis      
   }

   @Override
   public void removeFromBullet()
   {
      super.removeFromBullet();
      rockParentObject.removeFromBullet();
      rockChildObject.removeFromBullet();
      rockSliderObject.removeFromBullet();
   }

   @Override
   public void updateRenderablesPoses()
   {
      // do nothing
   }

   @Override
   public boolean intersect(Line3DReadOnly pickRay, Point3D intersectionToPack)
   {
      return rockParentObject.intersect(pickRay, intersectionToPack);
   }

   @Override
   public void setSelected(boolean selected)
   {
      rockParentObject.setSelected(selected);
   }

   @Override
   public void setPositionInWorld(Point3DReadOnly positionInWorld)
   {
      rockParentObject.setPositionInWorld(positionInWorld);
      rockChildObject.setPositionInWorld(new Point3D(positionInWorld.getX(), positionInWorld.getY(), positionInWorld.getZ() - 0.5));
      rockSliderObject.setPositionInWorld(new Point3D(positionInWorld.getX(), positionInWorld.getY(), positionInWorld.getZ() - 1.0));
   }

   @Override
   public void setPoseInWorld(Pose3D poseInWorld)
   {
      rockParentObject.setPoseInWorld(poseInWorld);
   }

   @Override
   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      rockParentObject.setTransformToWorld(transformToWorld);
   }

   @Override
   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      rockParentObject.getRealRenderables(renderables, pool);
      rockChildObject.getRealRenderables(renderables, pool);
      rockSliderObject.getRealRenderables(renderables, pool);
   }

   @Override
   public void getCollisionMeshRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      rockParentObject.getCollisionMeshRenderables(renderables, pool);
      rockChildObject.getCollisionMeshRenderables(renderables, pool);
      rockSliderObject.getCollisionMeshRenderables(renderables, pool);
   }

   @Override
   public RigidBodyTransform getObjectTransform()
   {
      return rockParentObject.getObjectTransform();
   }

}
