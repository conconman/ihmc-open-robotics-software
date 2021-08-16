package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.simulationconstructionset.KinematicPoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ContactPointParticleVisualizer
{
   public ContactPointParticleVisualizer()
   {
      MultiPendulumRobot robot = new MultiPendulumRobot("multiPendulum", 5);
      robot.setInitialState(0.2, -0.3, 0.1, 1.0, -0.4);

      int linkIndex = 3;
      Point3D pointOffset = new Point3D(0.05, 0.05, 0.05);

      KinematicPoint kinematicPoint = new KinematicPoint("kinematicPoint", pointOffset, robot);
      robot.getScsJoints()[linkIndex].addKinematicPoint(kinematicPoint);

      ContactPointParticle contactPointParticle = new ContactPointParticle("testParticle", robot.getJoints());
      RigidBodyBasics link = robot.getJoints()[linkIndex].getSuccessor();
      contactPointParticle.setRigidBody(link);

      YoRegistry registry = new YoRegistry("controller");
      YoFrameVector3D velocity = new YoFrameVector3D("kpVelocity", ReferenceFrame.getWorldFrame(), registry);
      contactPointParticle.getSurfaceNormal().set(Axis3D.Z);

      RobotController controller = new RobotController()
      {
         @Override
         public void initialize()
         {
         }

         @Override
         public void doControl()
         {
            robot.updateState();
            contactPointParticle.getContactPointPosition().setIncludingFrame(robot.getJoints()[linkIndex].getFrameAfterJoint(), pointOffset);
            contactPointParticle.computeContactJacobian();
            contactPointParticle.update();
            velocity.set(contactPointParticle.getContactPointVelocity());
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return registry;
         }
      };

      robot.setController(controller);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

      scs.setGroundVisible(false);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new ContactPointParticleVisualizer();
   }
}
