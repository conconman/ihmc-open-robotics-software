package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNative;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNativeInput;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;


public class CylinderAndPlaneContactForceOptimizerMatrixCalculator
{
   private static final boolean DEBUG = false;
   private static final int PHISIZE = CylinderAndPlaneContactForceOptimizerNative.phiSize;
   private static final int RHOSIZE = CylinderAndPlaneContactForceOptimizerNative.rhoSize;
   private final ReferenceFrame centerOfMassFrame;
   private final SpatialForceVector[] qRhoVectors;
   private final SpatialForceVector[] qPhiVectors;
   private final BooleanYoVariable debug;
   private final DynamicGraphicVector[][][] graphicWrenches = new DynamicGraphicVector[2][][];
   private final DoubleYoVariable[][][] graphicYoDoubles = new DoubleYoVariable[2][][];
   private final FramePoint tempPoint;
   private final FrameVector tempVector;
   private static final int X = 0;
   private static final int Y = 1;
   private static final int Z = 2;
   private static final int xx = 3;
   private static final int yy = 4;
   private static final int zz = 5;
   private static final int x = 6;
   private static final int y = 7;
   private static final int z = 8;
   private static final int LINEAR = 0;
   private static final int ANGULAR = 1;
   private ArrayList<DynamicGraphicObject> dynamicGraphicVectorsRhoLinear;
   private ArrayList<DynamicGraphicObject> dynamicGraphicVectorsRhoAngular;
   private ArrayList<DynamicGraphicObject> dynamicGraphicVectorsPhiLinear;
   private ArrayList<DynamicGraphicObject> dynamicGraphicVectorsPhiAngular;
   private final boolean visualize;



   public CylinderAndPlaneContactForceOptimizerMatrixCalculator(String name, ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      visualize = null != dynamicGraphicObjectsListRegistry;
      YoVariableRegistry registry = new YoVariableRegistry(name);
      parentRegistry.addChild(registry);
      this.centerOfMassFrame = centerOfMassFrame;
      this.tempPoint = new FramePoint(centerOfMassFrame);
      this.tempVector = new FrameVector(centerOfMassFrame);

      qRhoVectors = new SpatialForceVector[RHOSIZE];
      qPhiVectors = new SpatialForceVector[PHISIZE];

      for (int i = 0; i < RHOSIZE; i++)
      {
         qRhoVectors[i] = new SpatialForceVector(centerOfMassFrame);
      }

      for (int i = 0; i < PHISIZE; i++)
      {
         qPhiVectors[i] = new SpatialForceVector(centerOfMassFrame);
      }

      if (visualize)
      {
         graphicWrenches[0] = new DynamicGraphicVector[RHOSIZE][2];
         graphicYoDoubles[0] = new DoubleYoVariable[RHOSIZE][9];
         graphicWrenches[1] = new DynamicGraphicVector[PHISIZE][2];
         graphicYoDoubles[1] = new DoubleYoVariable[PHISIZE][9];
         dynamicGraphicVectorsRhoLinear = new ArrayList<DynamicGraphicObject>();
         dynamicGraphicVectorsRhoAngular = new ArrayList<DynamicGraphicObject>();
         dynamicGraphicVectorsPhiLinear = new ArrayList<DynamicGraphicObject>();
         dynamicGraphicVectorsPhiAngular = new ArrayList<DynamicGraphicObject>();

         double scaleFactor = 0.25;


         int q = 0;
         for (int i = 0; i < RHOSIZE; i++)
         {
            for (int j = 0; j < 9; j++)
            {
               graphicYoDoubles[q][i][j] = new DoubleYoVariable("rhoGraphicVectorElement" + q + i + j, registry);
            }

            double greenLevel = 0.25 * i / (double) RHOSIZE;
            graphicWrenches[q][i][LINEAR] = new DynamicGraphicVector("RhoGraphicBasis" + q + i + "Linear", graphicYoDoubles[q][i][X],
                    graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][x], graphicYoDoubles[q][i][y], graphicYoDoubles[q][i][z],
                    scaleFactor, new YoAppearanceRGBColor(0.5, greenLevel, 0.0, 0.7));
            dynamicGraphicVectorsRhoLinear.add(graphicWrenches[q][i][LINEAR]);
            graphicWrenches[q][i][ANGULAR] = new DynamicGraphicVector("RhoGraphicBasis" + q + i + "Angular", graphicYoDoubles[q][i][X],
                    graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][xx], graphicYoDoubles[q][i][yy], graphicYoDoubles[q][i][zz],
                    scaleFactor, new YoAppearanceRGBColor(0.5, greenLevel, 0.6, 0.7));
            dynamicGraphicVectorsRhoAngular.add(graphicWrenches[q][i][ANGULAR]);
         }

         q = 1;

         for (int i = 0; i < PHISIZE; i++)
         {
            double greenLevel = 0.25 * i / (double) PHISIZE;
            for (int j = 0; j < 9; j++)
            {
               graphicYoDoubles[q][i][j] = new DoubleYoVariable("rhoGraphicVectorElement" + q + i + j, registry);
            }

            graphicWrenches[q][i][LINEAR] = new DynamicGraphicVector( "PhiGraphicBasis" + q + i + "Linear", graphicYoDoubles[q][i][X],
                    graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][x], graphicYoDoubles[q][i][y], graphicYoDoubles[q][i][z],
                    scaleFactor, new YoAppearanceRGBColor(1.0, greenLevel, 0.0, 0.7));
            dynamicGraphicVectorsPhiLinear.add(graphicWrenches[q][i][LINEAR]);
            graphicWrenches[q][i][ANGULAR] = new DynamicGraphicVector("PhiGraphicBasis" + q + i + "Angular", graphicYoDoubles[q][i][X],
                    graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][xx], graphicYoDoubles[q][i][yy], graphicYoDoubles[q][i][zz],
                    scaleFactor, new YoAppearanceRGBColor(1.0, greenLevel, 0.6, 0.7));
            dynamicGraphicVectorsPhiAngular.add(graphicWrenches[q][i][ANGULAR]);

         }

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects("rawBasisVectorsRhoLinear ", dynamicGraphicVectorsRhoLinear);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects("rawBasisVectorsRhoAngular", dynamicGraphicVectorsRhoAngular);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects("rawBasisVectorsPhiLinear ", dynamicGraphicVectorsPhiLinear);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects("rawBasisVectorsPhiAngular", dynamicGraphicVectorsPhiAngular);
      }

      this.debug = new BooleanYoVariable(this.getClass().getSimpleName() + "Debug", registry);
      this.debug.set(DEBUG);

   }

   public void computeAllMatriciesAndPopulateNativeInput(Collection<? extends EndEffector> endEffectors,
           CylinderAndPlaneContactForceOptimizerNativeInput nativeInput)
   {
      int iRho = 0;
      int iPhi = 0;
      
      for (EndEffector endEffector : endEffectors)
      {
         if (endEffector.isLoadBearing())
         {
            OptimizerContactModel model = endEffector.getContactModel();

            for (int iRhoModel = 0; iRhoModel < model.getSizeInRho(); iRhoModel++)
            {
               SpatialForceVector currentBasisVector = qRhoVectors[iRho];

               nativeInput.setRhoMin(iRho, 0, model.getRhoMin(iRhoModel));
               model.packQRhoBodyFrame(iRhoModel, currentBasisVector, endEffector.getReferenceFrame());

               currentBasisVector.changeFrame(centerOfMassFrame);
               nativeInput.setQRho(iRho, currentBasisVector);

               iRho++;
            }

            for (int iPhiModel = 0; iPhiModel < model.getSizeInPhi(); iPhiModel++)
            {
               SpatialForceVector currentBasisVector = qPhiVectors[iPhi];

               nativeInput.setPhiMin(iPhi, 0, model.getPhiMin(iPhiModel));
               nativeInput.setPhiMax(iPhi, 0, model.getPhiMax(iPhiModel));
               model.packQPhiBodyFrame(iPhiModel, currentBasisVector, endEffector.getReferenceFrame());

               currentBasisVector.changeFrame(centerOfMassFrame);
               nativeInput.setQPhi(iPhi, currentBasisVector);

               iPhi++;
            }
         }
      }

      if (visualize)
      {
         iRho = 0;
         iPhi = 0;
         int q = 0;

         for (EndEffector endEffector : endEffectors)
         {
            if (endEffector.isLoadBearing())
            {
               ReferenceFrame frameOfInterest = endEffector.getReferenceFrame();
               OptimizerContactModel contactModel = endEffector.getContactModel();
               if (contactModel instanceof OptimizerCylinderContactModel)
               {
                  frameOfInterest = ((OptimizerCylinderContactModel) contactModel).getCylinderFrame();
               }

               tempPoint.setToZero(frameOfInterest);
               tempPoint.changeFrame(endEffector.getReferenceFrame().getRootFrame());
               q = 0;
               OptimizerContactModel model = contactModel;

               for (int iRhoModel = 0; iRhoModel < model.getSizeInRho(); iRhoModel++)
               {
                  SpatialForceVector currentBasisVector = qRhoVectors[iRho];
                  currentBasisVector.changeFrame(frameOfInterest);
                  packYoDoubles(iRho, q, currentBasisVector, tempPoint);
                  iRho++;
               }

               q = 1;

               for (int iPhiModel = 0; iPhiModel < model.getSizeInPhi(); iPhiModel++)
               {
                  SpatialForceVector currentBasisVector = qPhiVectors[iPhi];
                  currentBasisVector.changeFrame(frameOfInterest);
                  packYoDoubles(iPhi, q, currentBasisVector, tempPoint);
                  iPhi++;
               }
            }
         }
      }


   }

   private void packYoDoubles(int iPhi, int q, SpatialForceVector currentBasisVector, FramePoint localPoint)
   {
      graphicYoDoubles[q][iPhi][X].set(localPoint.getX());
      graphicYoDoubles[q][iPhi][Y].set(localPoint.getY());
      graphicYoDoubles[q][iPhi][Z].set(localPoint.getZ());
      tempVector.changeFrame(currentBasisVector.getExpressedInFrame());
      currentBasisVector.packAngularPart(tempVector);
      tempVector.changeFrame(currentBasisVector.getExpressedInFrame().getRootFrame());
      graphicYoDoubles[q][iPhi][xx].set(tempVector.getX());
      graphicYoDoubles[q][iPhi][yy].set(tempVector.getY());
      graphicYoDoubles[q][iPhi][zz].set(tempVector.getZ());
      tempVector.changeFrame(currentBasisVector.getExpressedInFrame());
      currentBasisVector.packLinearPart(tempVector);
      tempVector.changeFrame(currentBasisVector.getExpressedInFrame().getRootFrame());
      graphicYoDoubles[q][iPhi][x].set(tempVector.getX());
      graphicYoDoubles[q][iPhi][y].set(tempVector.getY());
      graphicYoDoubles[q][iPhi][z].set(tempVector.getZ());
   }

   public void printIfDebug(String message)
   {
      if (this.debug.getBooleanValue())
      {
         System.out.println(this.getClass().getSimpleName() + ": " + message);
      }
   }
}
