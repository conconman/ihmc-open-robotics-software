package us.ihmc.rdx.simulation.bullet;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.*;
import com.badlogic.gdx.physics.bullet.dynamics.*;
import com.badlogic.gdx.physics.bullet.linearmath.btMotionState;

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
   private final ArrayList<btRigidBody> rigidBodies = new ArrayList<>();
   private final ArrayList<btMultiBody> multiBodies = new ArrayList<>();
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

   public void addRigidBody(btCollisionShape collisionShape, float mass, Matrix4 transformToWorld)
   {
      btMotionState motionState = new btMotionState()
      {
         @Override
         public void getWorldTransform(Matrix4 worldTrans)
         {
            worldTrans.set(transformToWorld);
         }

         @Override
         public void setWorldTransform(Matrix4 worldTrans)
         {
            transformToWorld.set(worldTrans);
         }
      };
      addRigidBody(collisionShape, mass, motionState);
   }

   public btRigidBody addRigidBody(btCollisionShape collisionShape, float mass, btMotionState motionState)
   {
      Vector3 localInertia = new Vector3();
      collisionShape.calculateLocalInertia(mass, localInertia);
      btRigidBody rigidBody = new btRigidBody(mass, motionState, collisionShape, localInertia);
      int collisionGroup = 1; // group 1 is rigid and static bodies
      int collisionGroupMask = 1 + 2; // Allow interaction with group 2, which is multi bodies
      multiBodyDynamicsWorld.addRigidBody(rigidBody, collisionGroup, collisionGroupMask);
      rigidBodies.add(rigidBody);
      return rigidBody;
   }

   public void addMultiBody(btMultiBody multiBody)
   {
      multiBodyDynamicsWorld.addMultiBody(multiBody);
      multiBodies.add(multiBody);
   }

   public void addMultiBodyCollisionShape(btMultiBodyLinkCollider collisionShape)
   {
      int collisionGroup = 2; // Multi bodies need to be in a separate collision group
      int collisionGroupMask = 1 + 2; // But allowed to interact with group 1, which is rigid and static bodies
      multiBodyDynamicsWorld.addCollisionObject(collisionShape, collisionGroup, collisionGroupMask);
   }

   public void addPostTickRunnable(Runnable postTickRunnable)
   {
      postTickRunnables.add(postTickRunnable);
   }

   public void setKinematicObject(btRigidBody btRigidBody, boolean isKinematicObject)
   {
      if (isKinematicObject)
      {
         btRigidBody.setCollisionFlags(btRigidBody.getCollisionFlags() | btCollisionObject.CollisionFlags.CF_KINEMATIC_OBJECT);
         btRigidBody.setActivationState(CollisionConstants.DISABLE_DEACTIVATION);
      }
      else
      {
         btRigidBody.setCollisionFlags(btRigidBody.getCollisionFlags() & ~btCollisionObject.CollisionFlags.CF_KINEMATIC_OBJECT);
         btRigidBody.setActivationState(CollisionConstants.WANTS_DEACTIVATION);
      }
   }

   public void removeCollisionObject(btCollisionObject collisionObject)
   {
      multiBodyDynamicsWorld.removeCollisionObject(collisionObject);
      rigidBodies.remove(collisionObject);
   }

   public void removeMultiBody(btMultiBody btMultiBody)
   {
      multiBodyDynamicsWorld.removeMultiBody(btMultiBody);
      multiBodies.remove(btMultiBody);
   }

   public void destroy()
   {
      postTickRunnables.clear();
      for (btCollisionObject collisionObject : collisionObjects)
      {
         multiBodyDynamicsWorld.removeCollisionObject(collisionObject);
      }
      for (btRigidBody rigidBody : rigidBodies)
      {
         multiBodyDynamicsWorld.removeRigidBody(rigidBody);
      }
      for (btMultiBody multiBody : multiBodies)
      {
         multiBodyDynamicsWorld.removeMultiBody(multiBody);
      }
   }

   public btMultiBodyDynamicsWorld getMultiBodyDynamicsWorld()
   {
      return multiBodyDynamicsWorld;
   }
}
