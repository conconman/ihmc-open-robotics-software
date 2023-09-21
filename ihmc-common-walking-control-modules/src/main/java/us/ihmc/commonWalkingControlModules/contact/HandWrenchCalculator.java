package us.ihmc.commonWalkingControlModules.contact;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialVectorBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.math.filters.AlphaFilteredYoSpatialVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class HandWrenchCalculator
{
   private final YoRegistry registry;
   private final GeometricJacobianCalculator geometricJacobianCalculator = new GeometricJacobianCalculator();
   private final List<JointReadOnly> jointsFromBaseToEndEffector;
   private final List<OneDoFJointBasics> armJoints;
   private final SpatialVector rawWrench = new SpatialVector();
   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final double[] jointTorquesForGravity;
   private final double[] jointTorques;
   private final AlphaFilteredYoSpatialVector alphaFilteredYoSpatialVector;

   public HandWrenchCalculator(RobotSide side, FullHumanoidRobotModel fullRobotModel, YoRegistry parentRegistry, double expectedComputeDT)
   {
      registry = new YoRegistry(HandWrenchCalculator.class.getSimpleName() + side.getPascalCaseName());

      geometricJacobianCalculator.setKinematicChain(fullRobotModel.getChest(), fullRobotModel.getHand(side));
      geometricJacobianCalculator.setJacobianFrame(fullRobotModel.getHandControlFrame(side));

      jointsFromBaseToEndEffector = geometricJacobianCalculator.getJointsFromBaseToEndEffector();
      armJoints = MultiBodySystemTools.filterJoints(jointsFromBaseToEndEffector, OneDoFJointBasics.class);
      jointTorques = new double[armJoints.size()];
      jointTorquesForGravity = new double[armJoints.size()];

      inverseDynamicsCalculator = new InverseDynamicsCalculator(MultiBodySystemReadOnly.toMultiBodySystemInput(armJoints));
      inverseDynamicsCalculator.setConsiderCoriolisAndCentrifugalForces(false);
      inverseDynamicsCalculator.setGravitionalAcceleration(-9.81);

      // RobotConfigurationData is published at 120Hz -> break freq.: 5Hz - 20Hz
      double breakFrequency = 20;
      double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency, expectedComputeDT);
      SpatialVector spatialVectorForSetup = new SpatialVector();
      alphaFilteredYoSpatialVector = new AlphaFilteredYoSpatialVector("filteredWrench",
                                                                      side.toString(),
                                                                      registry,
                                                                      () -> alpha,
                                                                      () -> alpha,
                                                                      spatialVectorForSetup.getAngularPart(),
                                                                      spatialVectorForSetup.getLinearPart());
      parentRegistry.addChild(registry);
   }

   public void compute()
   {
      // Compute gravity compensation torques
      // TODO: Check if this looks good.
      //   Remove gravity compensation from the wrench when called? but gravity compensation would change as the arm moves...
      inverseDynamicsCalculator.compute();
      for (int i = 0; i < armJoints.size(); ++i)
      {
         DMatrixRMaj tau = inverseDynamicsCalculator.getComputedJointTau(armJoints.get(i));
         if (tau != null)
         {
            jointTorquesForGravity[i] = tau.get(0, 0);
         }
      }

      for (int i = 0; i < armJoints.size(); ++i)
      {
         jointTorques[i] = armJoints.get(i).getTau() - jointTorquesForGravity[i];
      }

      // Need to call rest to update the joint configurations in the jacobian calculator.
      geometricJacobianCalculator.reset();
      // getJacobianMatrix updates the matrix and outputs in the form of DMatrixRMaj
      DMatrixRMaj armJacobian = geometricJacobianCalculator.getJacobianMatrix();
      DMatrixRMaj armJacobianTransposed = CommonOps_DDRM.transpose(armJacobian, null);
      DMatrixRMaj armJacobianTransposedDagger = leftPseudoInverse(armJacobianTransposed);
      DMatrixRMaj jointTorqueVector = new DMatrixRMaj(jointTorques);
      DMatrixRMaj wrenchVector = new DMatrixRMaj(6, 1);
      CommonOps_DDRM.mult(armJacobianTransposedDagger, jointTorqueVector, wrenchVector);

      rawWrench.setReferenceFrame(geometricJacobianCalculator.getJacobianFrame());
      rawWrench.set(wrenchVector);
      rawWrench.changeFrame(ReferenceFrame.getWorldFrame());
      // Negate to express the wrench the hand experiences from the external world
      rawWrench.negate();

      alphaFilteredYoSpatialVector.update(rawWrench.getAngularPart(), rawWrench.getLinearPart());
   }

   private DMatrixRMaj leftPseudoInverse(DMatrixRMaj matrix)
   {
      DampedLeastSquaresSolver pseudoInverseSolver = new DampedLeastSquaresSolver(matrix.getNumRows(), 1e-6);
      pseudoInverseSolver.setA(matrix);
      DMatrixRMaj leftPseudoInverseOfMatrix = new DMatrixRMaj(matrix);
      pseudoInverseSolver.invert(leftPseudoInverseOfMatrix);
      return leftPseudoInverseOfMatrix;
   }

   public SpatialVector getUnfilteredWrench()
   {
      return rawWrench;
   }

   public AlphaFilteredYoSpatialVector getFilteredWrench()
   {
      return alphaFilteredYoSpatialVector;
   }

   public double getLinearWrenchMagnitude(boolean filtered)
   {
      FixedFrameSpatialVectorBasics wrench = filtered ? getFilteredWrench() : getUnfilteredWrench();
      return wrench.getLinearPart().norm();
   }

   public double getAngularWrenchMagnitude(boolean filtered)
   {
      FixedFrameSpatialVectorBasics wrench = filtered ? getFilteredWrench() : getUnfilteredWrench();
      return wrench.getAngularPart().norm();
   }
}