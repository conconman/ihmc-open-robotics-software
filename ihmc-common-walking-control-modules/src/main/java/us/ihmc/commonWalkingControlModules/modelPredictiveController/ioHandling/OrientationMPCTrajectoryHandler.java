package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.SE3MPCIndexHandler;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.math.trajectories.FixedFramePolynomialEstimator3D;
import us.ihmc.robotics.math.trajectories.core.FramePolynomial3D;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.FramePolynomial3DBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.awt.*;
import java.util.List;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

/**
 * This class is meant to handle the trajectory from the MPC module. It includes the trajectory for the full planning window, which is overwritten with the
 * solution for the planning window at the beginning.
 * <p>
 * It assembles all this solution into a continuous multi-segment trajectory for the center of mass, and a list of polynomials for the VRP.
 */
public class OrientationMPCTrajectoryHandler
{
   private final OrientationTrajectoryCalculator orientationInitializationCalculator;

   private final SE3MPCIndexHandler indexHandler;

   private final RecyclingArrayList<FrameOrientation3DBasics> desiredOrientation = new RecyclingArrayList<>(FrameQuaternion::new);
   private final RecyclingArrayList<FrameVector3DBasics> desiredAngularVelocity = new RecyclingArrayList<>(FrameVector3D::new);

   private final RecyclingArrayList<FrameQuaternionBasics> orientationSolution = new RecyclingArrayList<>(FrameQuaternion::new);
   private final RecyclingArrayList<FrameVector3DBasics> angularVelocitySolution = new RecyclingArrayList<>(FrameVector3D::new);

   private final MultipleWaypointsOrientationTrajectoryGenerator bodyOrientationTrajectory;

   private final MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomialEstimator3D> internalAngularMomentumTrajectory;
   private final YoDouble previewWindowEndTime;

   private final RecyclingArrayList<AxisAngleBasics> axisAngleErrorSolution = new RecyclingArrayList<>(AxisAngle::new);
   private final RecyclingArrayList<FrameVector3DBasics> angularVelocityErrorSolution = new RecyclingArrayList<>(FrameVector3D::new);

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   public OrientationMPCTrajectoryHandler(SE3MPCIndexHandler indexHandler)
   {
      this.indexHandler = indexHandler;

      previewWindowEndTime = new YoDouble("orientationPreviewWindowEndTime", registry);

      internalAngularMomentumTrajectory = new MultipleSegmentPositionTrajectoryGenerator<>("internalAngularMomentumTrajectory",
                                                                                   50,
                                                                                   worldFrame,
                                                                                   () -> new FixedFramePolynomialEstimator3D(worldFrame),
                                                                                   registry);

      orientationInitializationCalculator = new OrientationTrajectoryCalculator(registry);
      bodyOrientationTrajectory = new MultipleWaypointsOrientationTrajectoryGenerator("desiredCoMTrajectory", 100, ReferenceFrame.getWorldFrame(), registry);
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   /**
    * Clears the CoM and VRP solution trajectories
    */
   public void clearTrajectory()
   {
      previewWindowEndTime.set(Double.NEGATIVE_INFINITY);
      bodyOrientationTrajectory.clear();
   }


   public void extractSolutionForPreviewWindow(DMatrixRMaj solutionCoefficients,
                                               double currentTimeInState,
                                               double previewWindowDuration)
   {
      previewWindowEndTime.set(currentTimeInState + previewWindowDuration);
      extractSolutionVectors(solutionCoefficients);

      clearTrajectory();

      orientationSolution.clear();
      angularVelocitySolution.clear();

      int globalTick = 0;
      for (int segment = 0; segment < indexHandler.getNumberOfSegments(); segment++)
      {
         int end = globalTick + indexHandler.getOrientationTicksInSegment(segment);
         for (;globalTick < end; globalTick++)
         {
            currentTimeInState += indexHandler.getOrientationTickDuration(segment);

            FrameQuaternionBasics orientation = orientationSolution.add();
            orientation.set(desiredOrientation.get(globalTick));
            orientation.append(axisAngleErrorSolution.get(globalTick));

            FrameVector3DBasics angularVelocity = angularVelocitySolution.add();
            angularVelocity.set(angularVelocityErrorSolution.get(globalTick));

            orientation.transform(angularVelocity);
            angularVelocity.add(desiredAngularVelocity.get(globalTick));

            bodyOrientationTrajectory.appendWaypoint(currentTimeInState, orientation, angularVelocity);
         }
      }

      overwriteTrajectoryOutsidePreviewWindow();
   }

   private void extractSolutionVectors(DMatrixRMaj solutionCoefficients)
   {
      int totalNumberOfTicks = 0;
      for (int i = 0; i < indexHandler.getNumberOfSegments(); i++)
         totalNumberOfTicks += indexHandler.getOrientationTicksInSegment(i);

      axisAngleErrorSolution.clear();
      angularVelocityErrorSolution.clear();

      for (int i = 0; i < totalNumberOfTicks; i++)
      {
         AxisAngleBasics axisAngleError = axisAngleErrorSolution.add();
         FrameVector3DBasics angularVelocityError = angularVelocityErrorSolution.add();
         axisAngleError.setRotationVector(solutionCoefficients.get(indexHandler.getOrientationTickStartIndex(i), 0),
                                          solutionCoefficients.get(indexHandler.getOrientationTickStartIndex(i) + 1, 0),
                                          solutionCoefficients.get(indexHandler.getOrientationTickStartIndex(i) + 2, 0));
         angularVelocityError.set(solutionCoefficients.get(indexHandler.getOrientationTickStartIndex(i) + 3, 0),
                                  solutionCoefficients.get(indexHandler.getOrientationTickStartIndex(i) + 4, 0),
                                  solutionCoefficients.get(indexHandler.getOrientationTickStartIndex(i) + 5, 0));
      }
   }

   public void setInitialBodyOrientationState(FrameOrientation3DReadOnly bodyOrientation, FrameVector3DReadOnly bodyAngularVelocity)
   {
      orientationInitializationCalculator.setInitialBodyOrientation(bodyOrientation, bodyAngularVelocity);
   }

   public void solveForTrajectoryOutsidePreviewWindow(List<ContactPlaneProvider> fullContactSequence)
   {
      orientationInitializationCalculator.solveForTrajectory(fullContactSequence);

      removeInfoOutsidePreviewWindow();
      overwriteTrajectoryOutsidePreviewWindow();
   }

   public void computeDesiredTrajectory(double currentTimeInState)
   {
      desiredOrientation.clear();
      desiredAngularVelocity.clear();

      for (int segment = 0; segment < indexHandler.getNumberOfSegments(); segment++)
      {
         for (int i = 0; i < indexHandler.getOrientationTicksInSegment(segment); i++)
         {
            currentTimeInState += indexHandler.getOrientationTickDuration(segment);

            bodyOrientationTrajectory.compute(currentTimeInState);

            desiredOrientation.add().set(bodyOrientationTrajectory.getOrientation());
            desiredAngularVelocity.add().set(bodyOrientationTrajectory.getAngularVelocity());
         }
      }
   }

   private void removeInfoOutsidePreviewWindow()
   {
      while (bodyOrientationTrajectory.getCurrentNumberOfWaypoints() > 0 && bodyOrientationTrajectory.getLastWaypointTime() > previewWindowEndTime.getValue())
      {
         bodyOrientationTrajectory.removeLastWaypoint();
      }
   }

   private void overwriteTrajectoryOutsidePreviewWindow()
   {
      MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryOutsideWindow = orientationInitializationCalculator.getOrientationTrajectory();

      boolean hasTrajectoryAlready = bodyOrientationTrajectory.getCurrentNumberOfWaypoints() > 0;
      double existingEndTime = hasTrajectoryAlready ? bodyOrientationTrajectory.getLastWaypointTime() : 0.0;
      if (existingEndTime >= orientationTrajectoryOutsideWindow.getLastWaypointTime())
         return;

      int waypointIndexToAdd = getWaypointIndexAfterTime(existingEndTime + 1e-5, orientationInitializationCalculator.getOrientationTrajectory());
      if (waypointIndexToAdd == -1)
         return;

      for (; waypointIndexToAdd < orientationTrajectoryOutsideWindow.getCurrentNumberOfWaypoints(); waypointIndexToAdd++)
      {
         bodyOrientationTrajectory.appendWaypoint(orientationTrajectoryOutsideWindow.getWaypoint(waypointIndexToAdd));
      }

      bodyOrientationTrajectory.initialize();
   }

   private static int getWaypointIndexAfterTime(double time, MultipleWaypointsOrientationTrajectoryGenerator trajectory)
   {
      for (int i = 0; i < trajectory.getCurrentNumberOfWaypoints(); i++)
      {
         if (trajectory.getWaypoint(i).getTime() > time)
            return i;
      }

      return -1;
   }

   public void compute(double timeInPhase)
   {
      bodyOrientationTrajectory.compute(timeInPhase);
      orientationInitializationCalculator.compute(timeInPhase);
      if (!internalAngularMomentumTrajectory.isEmpty())
         internalAngularMomentumTrajectory.compute(timeInPhase);
   }

   public void computeOutsidePreview(double timeInPhase)
   {
      orientationInitializationCalculator.compute(timeInPhase);
   }

   public void setInternalAngularMomentumTrajectory(MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomialEstimator3D> trajectory)
   {
      internalAngularMomentumTrajectory.clear();
      for (int i = 0; i < trajectory.getCurrentNumberOfSegments(); i++)
         internalAngularMomentumTrajectory.appendSegment(trajectory.getSegment(i));
      internalAngularMomentumTrajectory.initialize();
   }

   public FrameOrientation3DReadOnly getDesiredBodyOrientationOutsidePreview()
   {
      return orientationInitializationCalculator.getDesiredOrientation();
   }

   public FrameVector3DReadOnly getDesiredBodyVelocityOutsidePreview()
   {
      return orientationInitializationCalculator.getDesiredAngularVelocity();
   }

   public FrameOrientation3DReadOnly getDesiredBodyOrientation()
   {
      return bodyOrientationTrajectory.getOrientation();
   }

   public FrameVector3DReadOnly getDesiredAngularVelocity()
   {
      return bodyOrientationTrajectory.getAngularVelocity();
   }

   public FrameVector3DReadOnly getDesiredAngularAcceleration()
   {
      return bodyOrientationTrajectory.getAngularAcceleration();
   }

   public boolean hasInternalAngularMomentum()
   {
      return internalAngularMomentumTrajectory.isEmpty();
   }

   public FramePoint3DReadOnly getDesiredInternalAngularMomentum()
   {
      return internalAngularMomentumTrajectory.getPosition();
   }

   public FrameVector3DReadOnly getDesiredInternalAngularMomentumRate()
   {
      return internalAngularMomentumTrajectory.getVelocity();
   }
}
