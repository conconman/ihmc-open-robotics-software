package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelWrapper;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.List;

public class InertialExtendedKalmanFilter
{
   private final YoBoolean enableFilter;

   private final FullHumanoidRobotModel actualRobotModel;
   private final List<? extends JointBasics> actualModelJoints;

   private final FullHumanoidRobotModel estimateRobotModel;
   private final List<? extends JointBasics> estimateModelJoints;

   private final JointTorqueRegressorCalculator jointTorqueRegressorCalculator;

   private final SideDependentList<? extends FootSwitchInterface> footSwitches;
   private final SideDependentList<DMatrixRMaj> contactWrenches;

   private final JointIndexHandler jointIndexHandler;
   private final SideDependentList<JointBasics[]> legJoints;
   private final SideDependentList<GeometricJacobian> compactContactJacobians;
   private final SideDependentList<DMatrixRMaj> fullContactJacobians;

   private final DMatrixRMaj wholeSystemTorques;

   /** What we do all the math in */
   private final DMatrixRMaj generalizedForcesContainer;

   private final List<RigidBodyInertialParameters> inertialParameters = new ArrayList<>();
   private final List<YoMatrix> inertialParametersPiBasisWatchers = new ArrayList<>();
   private final List<YoMatrix> inertialParametersThetaBasisWatchers = new ArrayList<>();
   private final DMatrixRMaj inertialParameterPiBasisContainer;
   private final DMatrixRMaj inertialParameterThetaBasisContainer;

   // DEBUG variables
   private static final boolean DEBUG = true;
   private final DMatrixRMaj perfectRegressor;
   private final DMatrixRMaj perfectParameters;

   private final YoMatrix residual;

   public InertialExtendedKalmanFilter(HighLevelHumanoidControllerToolbox toolbox, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      enableFilter = new YoBoolean("enableFilter", registry);
      enableFilter.set(false);

      actualRobotModel = toolbox.getFullRobotModel();
      actualModelJoints = actualRobotModel.getRootJoint().subtreeList();

      RigidBodyBasics clonedElevator = MultiBodySystemFactories.cloneMultiBodySystem(actualRobotModel.getElevator(),
                                                                                     actualRobotModel.getModelStationaryFrame(),
                                                                                     "_estimate");
      estimateRobotModel = new FullHumanoidRobotModelWrapper(clonedElevator, true);
      estimateModelJoints = estimateRobotModel.getRootJoint().subtreeList();

      int totalNumberOfDoFs = actualRobotModel.getRootJoint().getDegreesOfFreedom() + actualRobotModel.getOneDoFJoints().length;

      jointTorqueRegressorCalculator = new JointTorqueRegressorCalculator(estimateRobotModel.getElevator());
      jointTorqueRegressorCalculator.setGravitationalAcceleration(-toolbox.getGravityZ());

      this.footSwitches = toolbox.getFootSwitches();
      contactWrenches = new SideDependentList<>(new DMatrixRMaj(6 ,1),
                                                new DMatrixRMaj(6, 1));  // TODO(jfoster): magic numbers

      jointIndexHandler = new JointIndexHandler(actualRobotModel.getElevator().subtreeJointStream().toArray(JointBasics[]::new));
      legJoints = new SideDependentList<>();
      compactContactJacobians = new SideDependentList<>();
      fullContactJacobians = new SideDependentList<>();
      // NOTE: for the leg joints and compact jacobians, we use the actual robot model because it has the full model information, including all joint names
      for (RobotSide side : RobotSide.values)
      {
         legJoints.set(side, MultiBodySystemTools.createJointPath(actualRobotModel.getElevator(), actualRobotModel.getFoot(side)));
         compactContactJacobians.set(side, new GeometricJacobian(legJoints.get(side), footSwitches.get(side).getMeasurementFrame()));
         fullContactJacobians.set(side, new DMatrixRMaj(6, totalNumberOfDoFs));
      }

      wholeSystemTorques = new DMatrixRMaj(totalNumberOfDoFs, 1);

      generalizedForcesContainer = new DMatrixRMaj(totalNumberOfDoFs, 1);

      inertialParameterPiBasisContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);
      inertialParameterThetaBasisContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);
      for (RigidBodyBasics body : estimateRobotModel.getElevator().subtreeList())
      {
         if (body.getInertia() != null)
         {
            inertialParameters.add(new RigidBodyInertialParameters(body.getInertia()));

            inertialParametersPiBasisWatchers.add(new YoMatrix(body.getName() + "_PiBasis", RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1, getRowNames(ParameterRepresentation.PI_BASIS), registry));
            inertialParametersThetaBasisWatchers.add(new YoMatrix(body.getName() + "_ThetaBasis", RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1, getRowNames(ParameterRepresentation.THETA_BASIS), registry));

            inertialParameters.get(inertialParameters.size() - 1).getParameterVectorPiBasis(inertialParameterPiBasisContainer);
            inertialParametersPiBasisWatchers.get(inertialParametersPiBasisWatchers.size() - 1).set(inertialParameterPiBasisContainer);
            inertialParameters.get(inertialParameters.size() - 1).getParameterVectorThetaBasis(inertialParameterThetaBasisContainer);
            inertialParametersThetaBasisWatchers.get(inertialParametersThetaBasisWatchers.size() - 1).set(inertialParameterThetaBasisContainer);
         }
      }

      if (DEBUG)
      {
         perfectRegressor = new DMatrixRMaj(jointTorqueRegressorCalculator.getJointTorqueRegressorMatrix());
         perfectParameters = new DMatrixRMaj(jointTorqueRegressorCalculator.getParameterVector());

         String[] rowNames = new String[totalNumberOfDoFs];
         int index = 0;
         for (JointReadOnly joint : actualModelJoints)
         {
            if (joint.getDegreesOfFreedom() > 1)
            {
               for (int i = 0; i < joint.getDegreesOfFreedom(); i++)
                  rowNames[index + i] = joint.getName() + "_" + i;
            }
            else
            {
               rowNames[index] = joint.getName();
            }
            index += joint.getDegreesOfFreedom();
         }
         residual = new YoMatrix("residual", totalNumberOfDoFs, 1, rowNames, registry);
      }
   }

   public void update()
   {
      if (enableFilter.getBooleanValue())
      {
         updateEstimatedModelJointState();

         updateContactJacobians();
         updateContactWrenches();
         updateWholeSystemTorques();

         jointTorqueRegressorCalculator.compute();

         // Start with system torques
         generalizedForcesContainer.set(wholeSystemTorques);

         // Minus contribution from contact wrenches
         for (RobotSide side : RobotSide.values)
            CommonOps_DDRM.multAddTransA(fullContactJacobians.get(side), contactWrenches.get(side), generalizedForcesContainer);


         if(DEBUG)
         {
            DMatrixRMaj residualToPack = new DMatrixRMaj(generalizedForcesContainer);

            perfectRegressor.set(jointTorqueRegressorCalculator.getJointTorqueRegressorMatrix());
            CommonOps_DDRM.multAdd(-1.0, perfectRegressor, perfectParameters, residualToPack);
            residual.set(residualToPack);
         }

         updateWatchers();
      }
   }

   private void updateContactWrenches()
   {
      for (RobotSide side: RobotSide.values)
         footSwitches.get(side).getMeasuredWrench().get(contactWrenches.get(side));
   }

   private void updateEstimatedModelJointState()
   {
      for (JointStateType type : JointStateType.values())
         MultiBodySystemTools.copyJointsState(actualModelJoints, estimateModelJoints, type);
      estimateRobotModel.getRootJoint().updateFramesRecursively();
   }

   private void updateWholeSystemTorques()
   {
      actualRobotModel.getRootJoint().getJointTau(0, wholeSystemTorques);
      for (OneDoFJointReadOnly joint : actualRobotModel.getOneDoFJoints())
      {
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         joint.getJointTau(jointIndex, wholeSystemTorques);
      }
   }

   private void updateContactJacobians()
   {
      for (RobotSide side : RobotSide.values)
      {
         compactContactJacobians.get(side).compute();
         jointIndexHandler.compactBlockToFullBlock(legJoints.get(side), compactContactJacobians.get(side).getJacobianMatrix(), fullContactJacobians.get(side));
      }
   }

   private void updateWatchers()
   {
      for (int i = 0; i < inertialParameters.size(); i++)
      {
         inertialParameters.get(i).getParameterVectorPiBasis(inertialParameterPiBasisContainer);
         inertialParametersPiBasisWatchers.get(i).set(inertialParameterPiBasisContainer);

         inertialParameters.get(i).getParameterVectorThetaBasis(inertialParameterThetaBasisContainer);
         inertialParametersThetaBasisWatchers.get(i).set(inertialParameterThetaBasisContainer);
      }
   }

   private enum ParameterRepresentation
   {
      PI_BASIS, THETA_BASIS
   }

   private String[] getRowNames(ParameterRepresentation representation)
   {
      String[] rowNames = new String[RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY];
      switch (representation)
      {
      case PI_BASIS:
      {
         rowNames[0] = "m";
         rowNames[1] = "comX";
         rowNames[2] = "comY";
         rowNames[3] = "comZ";
         rowNames[4] = "Ixx";
         rowNames[5] = "Ixy";
         rowNames[6] = "Iyy";
         rowNames[7] = "Ixz";
         rowNames[8] = "Iyz";
         rowNames[9] = "Izz";
         return rowNames;
      }
      case THETA_BASIS:
      {
         rowNames[0] = "alpha";
         rowNames[1] = "dx";
         rowNames[2] = "dy";
         rowNames[3] = "dz";
         rowNames[4] = "sxy";
         rowNames[5] = "sxz";
         rowNames[6] = "syz";
         rowNames[7] = "tx";
         rowNames[8] = "ty";
         rowNames[9] = "tz";
         return rowNames;
      }
      default:
         throw new RuntimeException("Unhandled case: " + representation);
      }
   }
}
