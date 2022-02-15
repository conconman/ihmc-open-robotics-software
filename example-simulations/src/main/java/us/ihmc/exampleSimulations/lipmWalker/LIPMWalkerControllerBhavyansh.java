package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialBasics;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class LIPMWalkerControllerBhavyansh implements RobotController
{
   private final LIPMWalkerRobot robot;
   private YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private YoDouble t;

   private final YoDouble kpKnee = new YoDouble("kpKnee", registry);
   private final YoDouble kdKnee = new YoDouble("kdKnee", registry);
   private final YoDouble kpHip = new YoDouble("kpHip", registry);
   private final YoDouble kdHip = new YoDouble("kdHip", registry);

   private final YoDouble q_d_leftKnee = new YoDouble("q_d_leftKnee", registry);
   private final YoDouble q_d_rightKnee = new YoDouble("q_d_rightKnee", registry);
   private final YoDouble q_d_leftHip = new YoDouble("q_d_leftHip", registry);
   private final YoDouble q_d_rightHip = new YoDouble("q_d_rightHip", registry);

   private final YoDouble q_rightHipWorld = new YoDouble("q_rightHipWorld", registry);
   private final YoDouble q_leftHipWorld = new YoDouble("q_leftHipWorld", registry);

   private final YoDouble footXTarget = new YoDouble("footXTarget", registry);

   private final YoDouble comHeight = new YoDouble("comHeight", registry);

   private final YoDouble comXVelocity = new YoDouble("comXVelocity", registry);
   private final YoDouble comXPositionFromFoot = new YoDouble("comXPositionFromFoot", registry);

   private final SideDependentList<YoDouble> desiredKneeLengths = new SideDependentList<YoDouble>(q_d_leftKnee, q_d_rightKnee);
   private final SideDependentList<YoDouble> desiredHipAngles = new SideDependentList<YoDouble>(q_d_leftHip, q_d_rightHip);
   private final SideDependentList<YoDouble> worldHipAngles = new SideDependentList<YoDouble>(q_leftHipWorld, q_rightHipWorld);
   private final SideDependentList<StateMachine<States, State>> stateMachines;

   private final YoDouble desiredHeight = new YoDouble("desiredHeight", registry);
   private final YoDouble orbitalEnergy = new YoDouble("orbitalEnergy", registry);
   private final YoDouble strideLength = new YoDouble("strideLength", registry);
   private final YoDouble hipDiffAngle = new YoDouble("hipDiffAngle", registry);
   private final YoDouble swingTime = new YoDouble("swingTime", registry);

   private PolynomialBasics trajectorySwingHipPitch;
   private PolynomialBasics trajectorySwingKneeLength;
   private Vector3DReadOnly footLocation;

   private enum States
   {
      SUPPORT, SWING;
   }

   private double timeOfLastFootSwitch = 0.0;
   private boolean footLocationCalculated = false;
   private final double g = 9.81;
   private double desiredTopVelocity = 0.7;
   private double desiredEnergy = 0.5 * desiredTopVelocity * desiredTopVelocity;

   public LIPMWalkerControllerBhavyansh(LIPMWalkerRobot robot)
   {
      this.robot = robot;

      t = (YoDouble) robot.getRobot().findVariable("t");
      trajectorySwingHipPitch = new YoPolynomial("trajectorySwingHipAngle", 6, registry);
      trajectorySwingKneeLength = new YoPolynomial("trajectorySwingKneeLength", 6, registry);

      initialize();

      stateMachines = setupStateMachines();
   }

   @Override
   public void initialize()
   {

      kpKnee.set(1200.0);
      kdKnee.set(800.0);

      kpHip.set(700.0);
      kdHip.set(40.0);

      q_d_leftKnee.set(0.8);
      q_d_rightKnee.set(0.7);

      q_d_leftHip.set(0.0);
      q_d_rightHip.set(0.0);

      desiredHeight.set(0.8);

      strideLength.set(0.2);
      swingTime.set(0.4);


      footLocation = calculateStepLocation();
      footXTarget.set(footLocation.getX());

      calculateSwingTrajectories(RobotSide.RIGHT, swingTime.getValue() / 2);


   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         stateMachines.get(robotSide).doAction();
         stateMachines.get(robotSide).doTransitions();
      }
   }

   private void controlSupportLeg(RobotSide side)
   {
      double kneeLength = robot.getKneeLength(side);
      double kneeVelocity = robot.getKneeVelocity(side);

      //         double desiredKneeLength = desiredKneeLengths.get(side).getValue();

      Point3DReadOnly centerOfMassPosition = robot.getCenterOfMassPosition();
      Vector3DReadOnly centerOfMassVelocity = robot.getCenterOfMassVelocity();
      double mass = robot.getMass();

      /* Compute and set orbital energy YoDouble. Based on virtual spring-mass system. */
      comXVelocity.set(robot.getCenterOfMassVelocity().getX());
      comXPositionFromFoot.set(robot.getCenterOfMassXDistanceFromSupportFoot());

      double orbitalEnergyValue = 0.5 * comXVelocity.getValue() * comXVelocity.getValue()
                                  - 0.5 * g / desiredHeight.getDoubleValue() * comXPositionFromFoot.getValue() * comXPositionFromFoot.getValue();
      orbitalEnergy.set(orbitalEnergyValue);

      comHeight.set(centerOfMassPosition.getZ());

      double feedForwardSupportKneeForce = g * mass * kneeLength / centerOfMassPosition.getZ();
      double feedBackKneeForce =
            kpKnee.getValue() * (desiredHeight.getValue() - comHeight.getValue()) + kdKnee.getValue() * (0.0 - centerOfMassVelocity.getZ());
      robot.setKneeForce(side, feedForwardSupportKneeForce + feedBackKneeForce);
      robot.setHipTorque(side, 0.0);

      worldHipAngles.get(side).set(robot.getHipAngle(side) + robot.getBodyPitchAngle());
   }

   private void controlSwingLeg(RobotSide side, double timeInState)
   {
      double feedBackKneeForce;

      double hipAngle = robot.getHipAngle(side) + robot.getBodyPitchAngle();
      worldHipAngles.get(side).set(hipAngle);

      double supportHipAngle = robot.getHipAngle(side.getOppositeSide()) + robot.getBodyPitchAngle();
      double hipVelocity = robot.getHipVelocity(side);
      double kneeLength = robot.getKneeLength(side);
      double kneeVelocity = robot.getKneeVelocity(side);

      /* ----------------------------------- Compute and set knee force. ----------------------------------------------*/
      trajectorySwingKneeLength.compute(timeInState);
      double desiredKneeLength = trajectorySwingKneeLength.getValue();
      double desiredKneeVelocity = trajectorySwingKneeLength.getVelocity();
      feedBackKneeForce = 3000 * (desiredKneeLength - kneeLength) + 200 * (desiredKneeVelocity - kneeVelocity);
      desiredKneeLengths.get(side).set(desiredKneeLength);
      robot.setKneeForce(side, feedBackKneeForce);

      /* ----------------------------------- Compute and set hip torque. --------------------------------------------*/
      trajectorySwingHipPitch.compute(timeInState);
      double desiredHipAngle = trajectorySwingHipPitch.getValue();
      double desiredHipVelocity = trajectorySwingHipPitch.getVelocity();
      double feedBackHipTorque = kpHip.getValue() * (desiredHipAngle - hipAngle) + kdHip.getValue() * (desiredHipVelocity - hipVelocity);
      desiredHipAngles.get(side).set(desiredHipAngle);
      robot.setHipTorque(side, feedBackHipTorque);
   }

   Vector3DReadOnly calculateStepLocation()
   {
      double energy = orbitalEnergy.getValue();
      double x_final = strideLength.getValue() / 2 + desiredHeight.getValue() / (g * strideLength.getValue()) * (desiredEnergy - energy);

      LogTools.info("Xf: {} \tEnergy: {} \tDesiredEnergy: {}", x_final, energy, desiredEnergy);

      Vector3DReadOnly footLocation = new Vector3D(x_final, 0.0, 0.0);
      return footLocation;
   }

   private void calculateSwingTrajectories(RobotSide swingSide, double totalTime)
   {
      double hipAngle = worldHipAngles.get(swingSide).getValue();
      double kneeLength = robot.getKneeLength(swingSide);
      double desiredKneeLength = EuclidCoreTools.squareRoot(footLocation.lengthSquared() / 4 + desiredHeight.getValue() * desiredHeight.getValue());
      double desiredHipAngle = -EuclidCoreTools.atan2(footLocation.length() / 2, desiredHeight.getValue());
      trajectorySwingHipPitch.setQuintic(0.0, totalTime, hipAngle, 0.0, 0.0, desiredHipAngle, 0.0, 0.0);
      trajectorySwingKneeLength.setQuintic(0.0, totalTime, 0.75, 0.0, 0.0, desiredKneeLength + 0.01, 0.0, 0.0);

   }

   private SideDependentList<StateMachine<States, State>> setupStateMachines()
   {
      // States and Actions:
      StateMachineFactory<States, State> leftFactory = new StateMachineFactory<>(States.class);
      StateMachineFactory<States, State> rightFactory = new StateMachineFactory<>(States.class);

      leftFactory.setNamePrefix("leftState");
      rightFactory.setNamePrefix("rightState");

      leftFactory.setRegistry(registry);
      rightFactory.setRegistry(registry);

      leftFactory.buildClock(robot.getRobot().getYoTime());
      rightFactory.buildClock(robot.getRobot().getYoTime());

      // Left State Transitions:
      leftFactory.addTransition(States.SUPPORT, States.SWING, new HeelOffGroundCondition(RobotSide.LEFT));
      leftFactory.addTransition(States.SWING, States.SUPPORT, new HeelOnGroundCondition(RobotSide.LEFT));

      // Right State Transitions:
      rightFactory.addTransition(States.SUPPORT, States.SWING, new HeelOffGroundCondition(RobotSide.RIGHT));
      rightFactory.addTransition(States.SWING, States.SUPPORT, new HeelOnGroundCondition(RobotSide.RIGHT));

      // Assemble the Left State Machine:
      leftFactory.addState(States.SUPPORT, new SupportState(RobotSide.LEFT));
      leftFactory.addState(States.SWING, new SwingState(RobotSide.LEFT));

      // Assemble the Right State Machine:
      rightFactory.addState(States.SUPPORT, new SupportState(RobotSide.RIGHT));
      rightFactory.addState(States.SWING, new SwingState(RobotSide.RIGHT));

      return new SideDependentList<>(leftFactory.build(States.SUPPORT), rightFactory.build(States.SWING));
   }

   private class SupportState implements State
   {
      private final RobotSide robotSide;

      public SupportState(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public void onEntry()
      {

      }

      @Override
      public void doAction(double timeInState)
      {
         // Support Side:
         controlSupportLeg(robotSide);
      }

      @Override
      public void onExit(double timeInState)
      {

      }
   }

   private class SwingState implements State
   {
      private final RobotSide robotSide;

      public SwingState(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public void onEntry()
      {

      }

      @Override
      public void doAction(double timeInState)
      {
         controlSwingLeg(robotSide, timeInState);
      }

      @Override
      public void onExit(double timeInState)
      {

      }
   }

   private class HeelOffGroundCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;

      public HeelOffGroundCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public boolean testCondition(double timeInCurrentState)
      {
         return robot.getFootSwitch(robotSide.getOppositeSide()) && (t.getValue() - timeOfLastFootSwitch > 0.1);
      }
   }

   private class HeelOnGroundCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;

      public HeelOnGroundCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public boolean testCondition(double timeInCurrentState)
      {
         double timeDiff = t.getValue() - timeOfLastFootSwitch;
         boolean fs = robot.getFootSwitch(robotSide) && timeDiff > 0.1; // Eliminates switch bouncing.
         if (fs)
         {
            footLocation = calculateStepLocation();
            footXTarget.set(footLocation.getX());

            calculateSwingTrajectories(robotSide.getOppositeSide(), swingTime.getValue());

            timeOfLastFootSwitch = t.getValue();
         }
         return fs;
      }
   }
}
