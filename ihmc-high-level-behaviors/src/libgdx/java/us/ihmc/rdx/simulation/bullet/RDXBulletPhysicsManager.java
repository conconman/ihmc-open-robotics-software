package us.ihmc.rdx.simulation.bullet;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.*;
import com.badlogic.gdx.physics.bullet.dynamics.*;

import java.util.ArrayList;

public class RDXBulletPhysicsManager
{
   static
   {
      RDXBulletTools.ensureBulletInitialized();
   }
   private btCollisionConfiguration collisionConfiguration;
   private btCollisionDispatcher collisionDispatcher;
   private btBroadphaseInterface broadphase;
   private btMultiBodyConstraintSolver solver;
   private btMultiBodyDynamicsWorld multiBodyDynamicsWorld;
   private final ArrayList<btCollisionObject> collisionObjects = new ArrayList<>(); // static, massless
   /**
    * Say we call bullet simulate and wait around for a while before calling it again.
    * The next time we call Bullet, it is going to try to catch back up by performing
    * several simulation ticks. If we wait like, 5 minutes, we surely don't want Bullet
    * to try and to 5 minutes of simulation in one tick. We probably never want it to do
    * more than like 1/4 of a second.
    */
   private final ArrayList<Runnable> postTickRunnables = new ArrayList<>();

   public void create()
   {
      collisionConfiguration = new btDefaultCollisionConfiguration();
      collisionDispatcher = new btCollisionDispatcher(collisionConfiguration);
      broadphase = new btDbvtBroadphase();
      solver = new btMultiBodyConstraintSolver();
      multiBodyDynamicsWorld = new btMultiBodyDynamicsWorld(collisionDispatcher, broadphase, solver, collisionConfiguration);
      Vector3 gravity = new Vector3(0.0f, 0.0f, -9.81f);
      multiBodyDynamicsWorld.setGravity(gravity);

      // Note: Apparently you can't have both pre- and post-tick callbacks, so we'll just do with post
      new InternalTickCallback(multiBodyDynamicsWorld, false)
      {
         @Override
         public void onInternalTick(btDynamicsWorld dynamicsWorld, float timeStep)
         {
            for (Runnable postTickRunnable : postTickRunnables)
            {
               postTickRunnable.run();
            }
         }
      };
   }

   public void addStaticObject(btCollisionShape collisionShape, Matrix4 transformToWorld)
   {
      btCollisionObject staticObject = new btCollisionObject();
      staticObject.setCollisionShape(collisionShape);
      staticObject.setWorldTransform(transformToWorld);
      multiBodyDynamicsWorld.addCollisionObject(staticObject);
      collisionObjects.add(staticObject);
   }

   public void destroy()
   {
      postTickRunnables.clear();
      for (btCollisionObject collisionObject : collisionObjects)
      {
         multiBodyDynamicsWorld.removeCollisionObject(collisionObject);
      }
   }

   public btMultiBodyDynamicsWorld getMultiBodyDynamicsWorld()
   {
      return multiBodyDynamicsWorld;
   }
}
