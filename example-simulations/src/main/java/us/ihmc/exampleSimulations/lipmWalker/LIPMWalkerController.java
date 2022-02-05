package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class LIPMWalkerController implements RobotController
{
   private final LIPMWalkerRobot robot;
   private YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble kpKnee = new YoDouble("kpKnee", registry);
   private final YoDouble kdKnee = new YoDouble("kdKnee", registry);
   private final YoDouble kpHip = new YoDouble("kpHip", registry);
   private final YoDouble kdHip = new YoDouble("kdHip", registry);

   private final YoDouble q_d_leftKnee = new YoDouble("q_d_leftKnee", registry);
   private final YoDouble q_d_rightKnee = new YoDouble("q_d_rightKnee", registry);
   private final YoDouble q_d_leftHip = new YoDouble("q_d_leftHip", registry);
   private final YoDouble q_d_rightHip = new YoDouble("q_d_rightHip", registry);

   private final YoDouble comHeight = new YoDouble("comHeight", registry);

   private final SideDependentList<YoDouble> desiredKneeLengths = new SideDependentList<YoDouble>(q_d_leftKnee, q_d_rightKnee);
   private final SideDependentList<YoDouble> desiredHipAngles = new SideDependentList<YoDouble>(q_d_leftHip, q_d_rightHip);

   private final YoDouble desiredHeight = new YoDouble("desiredHeight", registry);

   private final YoDouble orbitalEnergy = new YoDouble("orbitalEnergy", registry);

   private final double g  = 9.81;

   public LIPMWalkerController(LIPMWalkerRobot robot)
   {
      this.robot = robot;
      initialize();
   }

   @Override
   public void initialize()
   {
      kpKnee.set(1000.0);
      kdKnee.set(100.0);

      kpHip.set(500.0);
      kdHip.set(50.0);

      q_d_leftKnee.set(1.0);
      q_d_rightKnee.set(1.0);

      q_d_leftHip.set(0.0);
      q_d_rightHip.set(0.0);

      desiredHeight.set(0.8);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
      RobotSide side = RobotSide.LEFT;

      // Support Side:
      double kneeLength = robot.getKneeLength(side);
      double kneeVelocity = robot.getKneeVelocity(side);

      //         double desiredKneeLength = desiredKneeLengths.get(side).getValue();

      Point3DReadOnly centerOfMassPosition = robot.getCenterOfMassPosition();
      Vector3DReadOnly centerOfMassVelocity = robot.getCenterOfMassVelocity();
      double mass = robot.getMass();

      /* Compute and set orbital energy YoDouble. Based on virtual spring-mass system. */
      double orbitalEnergyValue = 0.5 * mass * centerOfMassVelocity.lengthSquared() - 0.5 * g / desiredHeight.getDoubleValue() * centerOfMassPosition.distanceFromOriginSquared();
      orbitalEnergy.set(orbitalEnergyValue);

      comHeight.set(centerOfMassPosition.getZ());

      double feedForwardSupportKneeForce = g * mass * kneeLength / centerOfMassPosition.getZ();
      double feedBackKneeForce =
            kpKnee.getValue() * (desiredHeight.getValue() - comHeight.getValue()) + kdKnee.getValue() * (0.0 - centerOfMassVelocity.getZ());
      robot.setKneeForce(side, feedForwardSupportKneeForce + feedBackKneeForce);

      // Swing Side:
      side = RobotSide.RIGHT;

      Vector3DReadOnly footLocation = calculateStepLocation();

      /* Compute knee force. */
      kneeLength = robot.getKneeLength(side);
      kneeVelocity = robot.getKneeVelocity(side);
      double desiredKneeLength = EuclidCoreTools.squareRoot(footLocation.lengthSquared() / 4 + desiredHeight.getValue() * desiredHeight.getValue());
      feedBackKneeForce = kpKnee.getValue() * (desiredKneeLength - kneeLength) + kdKnee.getValue() * (0.0 - kneeVelocity);
      robot.setKneeForce(side, feedBackKneeForce);

      /* Compute hip torque. */
      double hipAngle = robot.getHipAngle(side);
      double hipVelocity = robot.getHipVelocity(side);
      double desiredHipAngle = -2 * EuclidCoreTools.atan2(footLocation.length() / 2, desiredHeight.getValue());
      double feedBackHipTorque = kpHip.getValue() * (-Math.PI / 2 - hipAngle) + kdHip.getValue() * (0.0 - hipVelocity);
      robot.setHipTorque(side, feedBackHipTorque);
   }

   Vector3DReadOnly calculateStepLocation()
   {
      Vector3DReadOnly footLocation = new Vector3D(1.0, 0.0, 0.0);
      return footLocation;
   }

}
