package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.MatrixMissingTools;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCQPInputCalculator.sufficientlyLargeValue;
import static us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCQPInputCalculator.sufficientlyLongTime;

public class IntegrationInputCalculator
{
   public static void computeRhoAccelerationIntegrationMatrix(int startCol,
                                                              DMatrixRMaj gradientToPack,
                                                              DMatrixRMaj hessianToPack,
                                                              MPCContactPlane contactPlane,
                                                              double duration,
                                                              double omega,
                                                              double goalValueForPlane)
   {
      computeRhoAccelerationIntegrationMatrix(startCol, gradientToPack, hessianToPack, contactPlane.getRhoSize(), duration, omega, goalValueForPlane);
   }

   public static void computeRhoAccelerationIntegrationMatrix(int startCol,
                                                              DMatrixRMaj gradientToPack,
                                                              DMatrixRMaj hessianToPack,
                                                              MPCContactPoint contactPoint,
                                                              double duration,
                                                              double omega,
                                                              double goalValueForPlane)
   {
      computeRhoAccelerationIntegrationMatrix(startCol, gradientToPack, hessianToPack, contactPoint.getRhoSize(), duration, omega, goalValueForPlane);
   }

   public static void computeRhoAccelerationIntegrationMatrix(int startCol,
                                                              DMatrixRMaj gradientToPack,
                                                              DMatrixRMaj hessianToPack,
                                                              int numberOfBasisVectors,
                                                              double duration,
                                                              double omega,
                                                              double goalValueForBasis)
   {
      duration = Math.min(duration, sufficientlyLongTime);

      double positiveExponential = Math.min(Math.exp(omega * duration), sufficientlyLargeValue);
      double positiveExponential2 = Math.min(positiveExponential * positiveExponential, sufficientlyLargeValue);
      double negativeExponential = 1.0 / positiveExponential;
      double negativeExponential2 = negativeExponential * negativeExponential;
      double duration2 = duration * duration;
      double duration3 = duration * duration2;
      double omega2 = omega * omega;
      double omega3 = omega * omega2;
      double omega4 = omega2 * omega2;

      double c00 = omega3 / 2.0 * (positiveExponential2 - 1.0);
      double c01 = omega4 * duration;
      double c02 = 6.0 * (positiveExponential * (omega * duration - 1.0) + 1.0);
      double c03 = 2.0 * omega * (positiveExponential - 1.0);
      double c11 = -omega3 / 2.0 * (negativeExponential2 - 1.0);
      double c12 = -6.0 * (negativeExponential * (omega * duration + 1.0) - 1.0);
      double c13 = -2.0 * omega * (negativeExponential - 1.0);
      double c22 = 12.0 * duration3;
      double c23 = 6.0 * duration2;
      double c33 = 4.0 * duration;

      double g0 = omega * (positiveExponential - 1.0) * goalValueForBasis;
      double g1 = -omega * (negativeExponential - 1.0) * goalValueForBasis;
      double g2 = 3.0 * duration2 * goalValueForBasis;
      double g3 = 2.0 * duration * goalValueForBasis;

      for (int basisVectorIndexI = 0; basisVectorIndexI < numberOfBasisVectors; basisVectorIndexI++)
      {
         hessianToPack.unsafe_set(startCol, startCol, c00);
         hessianToPack.unsafe_set(startCol, startCol + 1, c01);
         hessianToPack.unsafe_set(startCol, startCol + 2, c02);
         hessianToPack.unsafe_set(startCol, startCol + 2, c03);
         hessianToPack.unsafe_set(startCol + 1, startCol, c01);
         hessianToPack.unsafe_set(startCol + 1, startCol + 1, c11);
         hessianToPack.unsafe_set(startCol + 1, startCol + 2, c12);
         hessianToPack.unsafe_set(startCol + 1, startCol + 3, c13);
         hessianToPack.unsafe_set(startCol + 2, startCol, c02);
         hessianToPack.unsafe_set(startCol + 2, startCol + 1, c12);
         hessianToPack.unsafe_set(startCol + 2, startCol + 2, c22);
         hessianToPack.unsafe_set(startCol + 2, startCol + 3, c23);
         hessianToPack.unsafe_set(startCol + 3, startCol, c03);
         hessianToPack.unsafe_set(startCol + 3, startCol + 1, c13);
         hessianToPack.unsafe_set(startCol + 3, startCol + 2, c23);
         hessianToPack.unsafe_set(startCol + 3, startCol + 3, c33);

         gradientToPack.unsafe_set(startCol, 0, -g0);
         gradientToPack.unsafe_set(startCol + 1, 0, -g1);
         gradientToPack.unsafe_set(startCol + 2, 0, -g2);
         gradientToPack.unsafe_set(startCol + 3, 0, -g3);

         startCol += LinearMPCIndexHandler.coefficientsPerRho;
      }
   }

   public static void computeNormalAccelerationIntegrationMatrix(int startCol,
                                                                 DMatrixRMaj gradientToPack,
                                                                 DMatrixRMaj hessianToPack,
                                                                 MPCContactPoint contactPoint,
                                                                 double duration,
                                                                 double omega,
                                                                 double goalNormalForce)
   {
      duration = Math.min(duration, sufficientlyLongTime);

      double positiveExponential = Math.min(Math.exp(omega * duration), sufficientlyLargeValue);
      double positiveExponential2 = Math.min(positiveExponential * positiveExponential, sufficientlyLargeValue);
      double negativeExponential = 1.0 / positiveExponential;
      double negativeExponential2 = negativeExponential * negativeExponential;
      double duration2 = duration * duration;
      double duration3 = duration * duration2;
      double omega2 = omega * omega;
      double omega3 = omega * omega2;
      double omega4 = omega2 * omega2;
      double c00 = omega3 / 2.0 * (positiveExponential2 - 1.0);
      double c01 = omega4 * duration;
      double c02 = 6.0 * (positiveExponential * (omega * duration - 1.0) + 1.0);
      double c03 = 2.0 * omega * (positiveExponential - 1.0);
      double c11 = -omega3 / 2.0 * (negativeExponential2 - 1.0);
      double c12 = -6.0 * (negativeExponential * (omega * duration + 1.0) - 1.0);
      double c13 = -2.0 * omega * (negativeExponential - 1.0);
      double c22 = 12.0 * duration3;
      double c23 = 6.0 * duration2;
      double c33 = 4.0 * duration;

      double g0 = omega * (positiveExponential - 1.0);
      double g1 = -omega * (negativeExponential - 1.0);
      double g2 = 3.0 * duration2;
      double g3 = 2.0 * duration;

      for (int basisVectorIndexI = 0; basisVectorIndexI < contactPoint.getRhoSize(); basisVectorIndexI++)
      {
         int startIdxI = startCol + basisVectorIndexI * LinearMPCIndexHandler.coefficientsPerRho;

         FrameVector3DReadOnly basisVectorI = contactPoint.getBasisVectorInPlaneFrame(basisVectorIndexI);

         hessianToPack.unsafe_set(startIdxI, startIdxI, c00);
         hessianToPack.unsafe_set(startIdxI, startIdxI + 1, c01);
         hessianToPack.unsafe_set(startIdxI, startIdxI + 2, c02);
         hessianToPack.unsafe_set(startIdxI, startIdxI + 2, c03);
         hessianToPack.unsafe_set(startIdxI + 1, startIdxI, c01);
         hessianToPack.unsafe_set(startIdxI + 1, startIdxI + 1, c11);
         hessianToPack.unsafe_set(startIdxI + 1, startIdxI + 2, c12);
         hessianToPack.unsafe_set(startIdxI + 1, startIdxI + 3, c13);
         hessianToPack.unsafe_set(startIdxI + 2, startIdxI, c02);
         hessianToPack.unsafe_set(startIdxI + 2, startIdxI + 1, c12);
         hessianToPack.unsafe_set(startIdxI + 2, startIdxI + 2, c22);
         hessianToPack.unsafe_set(startIdxI + 2, startIdxI + 3, c23);
         hessianToPack.unsafe_set(startIdxI + 3, startIdxI, c03);
         hessianToPack.unsafe_set(startIdxI + 3, startIdxI + 1, c13);
         hessianToPack.unsafe_set(startIdxI + 3, startIdxI + 2, c23);
         hessianToPack.unsafe_set(startIdxI + 3, startIdxI + 3, c33);

         for (int basisVectorIndexJ = basisVectorIndexI + 1; basisVectorIndexJ < contactPoint.getRhoSize(); basisVectorIndexJ++)
         {
            FrameVector3DReadOnly basisVectorJ = contactPoint.getBasisVectorInPlaneFrame(basisVectorIndexJ);

            double basisDot = basisVectorI.dot(basisVectorJ);

            int startIdxJ = startCol + basisVectorIndexJ * LinearMPCIndexHandler.coefficientsPerRho;

            MatrixMissingTools.unsafe_add(hessianToPack, startIdxI, startIdxJ, basisDot * c00);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxI, startIdxJ + 1, basisDot * c01);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxI, startIdxJ + 2, basisDot * c02);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxI, startIdxJ + 3, basisDot * c03);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxI + 1, startIdxJ, basisDot * c01);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxI + 1, startIdxJ + 1, basisDot * c11);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxI + 1, startIdxJ + 2, basisDot * c12);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxI + 1, startIdxJ + 3, basisDot * c13);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxI + 2, startIdxJ, basisDot * c02);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxI + 2, startIdxJ + 1, basisDot * c12);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxI + 2, startIdxJ + 2, basisDot * c22);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxI + 2, startIdxJ + 3, basisDot * c23);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxI + 3, startIdxJ, basisDot * c03);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxI + 3, startIdxJ + 1, basisDot * c13);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxI + 3, startIdxJ + 2, basisDot * c23);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxI + 3, startIdxJ + 3, basisDot * c33);

            // we know it's symmetric, and this way we can avoid iterating as much
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxJ, startIdxI, basisDot * c00);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxJ, startIdxI + 1, basisDot * c01);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxJ, startIdxI + 2, basisDot * c02);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxJ, startIdxI + 3, basisDot * c03);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxJ + 1, startIdxI, basisDot * c01);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxJ + 1, startIdxI + 1, basisDot * c11);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxJ + 1, startIdxI + 2, basisDot * c12);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxJ + 1, startIdxI + 3, basisDot * c13);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxJ + 2, startIdxI, basisDot * c02);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxJ + 2, startIdxI + 1, basisDot * c12);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxJ + 2, startIdxI + 2, basisDot * c22);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxJ + 2, startIdxI + 3, basisDot * c23);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxJ + 3, startIdxI, basisDot * c03);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxJ + 3, startIdxI + 1, basisDot * c13);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxJ + 3, startIdxI + 2, basisDot * c23);
            MatrixMissingTools.unsafe_add(hessianToPack, startIdxJ + 3, startIdxI + 3, basisDot * c33);
         }

         double basisValue = basisVectorI.getZ() * goalNormalForce;

         gradientToPack.unsafe_set(startIdxI, 0, -g0 * basisValue);
         gradientToPack.unsafe_set(startIdxI + 1, 0, -g1 * basisValue);
         gradientToPack.unsafe_set(startIdxI + 2, 0, -g2 * basisValue);
         gradientToPack.unsafe_set(startIdxI + 3, 0, -g3 * basisValue);
      }
   }
}
