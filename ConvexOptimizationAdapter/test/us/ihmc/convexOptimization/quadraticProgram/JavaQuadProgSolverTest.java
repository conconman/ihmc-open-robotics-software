package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.testing.JUnitTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class JavaQuadProgSolverTest
{
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testSolveContrainedQP() throws NoConvergenceException
   {
      int numberOfInequalityConstraints = 1;
      int numberOfEqualityConstraints = 1;
      int numberOfVariables = 2;

      DenseMatrix64F Q = new DenseMatrix64F(numberOfVariables, numberOfVariables, true, 1, 0, 0, 1);
      DenseMatrix64F f = new DenseMatrix64F(numberOfVariables, 1, true, 1, 0);
      DenseMatrix64F Aeq = new DenseMatrix64F(numberOfEqualityConstraints, numberOfVariables, true, 1, 1);
      DenseMatrix64F beq = new DenseMatrix64F(numberOfEqualityConstraints, 1, true, 0);
      DenseMatrix64F Ain = new DenseMatrix64F(numberOfInequalityConstraints, numberOfVariables, true, 2, 1);
      DenseMatrix64F bin = new DenseMatrix64F(numberOfInequalityConstraints, 1, true, 0);

      JavaQuadProgSolver solver = new JavaQuadProgSolver();

      for (int repeat = 0; repeat < 10000; repeat++)
      {
         DenseMatrix64F x = new DenseMatrix64F(numberOfVariables, 1, true, -1, 1);
         solver.solve(Q, f, Aeq, beq, Ain, bin, x, false);
         Assert.assertArrayEquals(x.getData(), new double[] { -0.5, 0.5 }, 1e-10);
      }

      numberOfInequalityConstraints = 1;
      numberOfEqualityConstraints = 2;
      numberOfVariables = 3;

      Q = new DenseMatrix64F(numberOfVariables, numberOfVariables, true, 1, 0, 1, 0, 1, 2, 1, 3, 7);
      f = new DenseMatrix64F(numberOfVariables, 1, true, 1, 0, 9);
      Aeq = new DenseMatrix64F(numberOfEqualityConstraints, numberOfVariables, true, 1, 1, 1, 2, 3, 4);
      beq = new DenseMatrix64F(numberOfEqualityConstraints, 1, true, 0, 7);
      Ain = new DenseMatrix64F(numberOfInequalityConstraints, numberOfVariables, true, 2, 1, 3);
      bin = new DenseMatrix64F(numberOfInequalityConstraints, 1, true, 0);

      for (int repeat = 0; repeat < 10000; repeat++)
      {
         DenseMatrix64F x = new DenseMatrix64F(numberOfVariables, 1, true, -1, 1, 3);
         solver.solve(Q, f, Aeq, beq, Ain, bin, x, false);

         DenseMatrix64F bEqualityVerify = new DenseMatrix64F(numberOfEqualityConstraints, 1);
         CommonOps.mult(Aeq, x, bEqualityVerify);

         // Verify Ax=b Equality constraints hold:
         JUnitTools.assertMatrixEquals(bEqualityVerify, beq, 1e-7);

         // Verify Ax<b Inequality constraints hold:
         DenseMatrix64F bInequalityVerify = new DenseMatrix64F(numberOfInequalityConstraints, 1);
         CommonOps.mult(Ain, x, bInequalityVerify);

         for (int j=0; j<bInequalityVerify.getNumRows(); j++)
         {
            Assert.assertTrue(bInequalityVerify.get(j, 0) < beq.get(j, 0));
         }

         // Verify solution is as expected
         Assert.assertArrayEquals("repeat = " + repeat + ", iterations = " + solver.getNumberOfIterations(), x.getData(), new double[] { -7.75, 8.5, -0.75 }, 1e-10);
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testTimingAgainstStandardQuadProg() throws NoConvergenceException
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      ExecutionTimer javaSolverTimer = new ExecutionTimer("javaSolverTimer", registry);
      ExecutionTimer wrapperSolverTimer = new ExecutionTimer("wrapperSolverTimer", registry);

      int numberOfInequalityConstraints = 1;
      int numberOfEqualityConstraints = 2;
      int numberOfVariables = 3;

      DenseMatrix64F Q = new DenseMatrix64F(numberOfVariables, numberOfVariables, true, 1, 0, 1, 0, 1, 2, 1, 3, 7);
      DenseMatrix64F f = new DenseMatrix64F(numberOfVariables, 1, true, 1, 0, 9);
      DenseMatrix64F Aeq = new DenseMatrix64F(numberOfEqualityConstraints, numberOfVariables, true, 1, 1, 1, 2, 3, 4);
      DenseMatrix64F beq = new DenseMatrix64F(numberOfEqualityConstraints, 1, true, 0, 7);
      DenseMatrix64F Ain = new DenseMatrix64F(numberOfInequalityConstraints, numberOfVariables, true, 2, 1, 3);
      DenseMatrix64F bin = new DenseMatrix64F(numberOfInequalityConstraints, 1, true, 0);

      for (int repeat = 0; repeat < 10000; repeat++)
      {
         DenseMatrix64F x = new DenseMatrix64F(numberOfVariables, 1, true, -1, 1, 3);
         DenseMatrix64F xWrapper = new DenseMatrix64F(numberOfVariables, 1, true, -1, 1, 3);

         JavaQuadProgSolver javaSolver = new JavaQuadProgSolver();
         javaSolverTimer.startMeasurement();
         javaSolver.solve(Q, f, Aeq, beq, Ain, bin, x, false);
         javaSolverTimer.stopMeasurement();

         QuadProgSolver solver = new QuadProgSolver();
         wrapperSolverTimer.startMeasurement();
         solver.solve(Q, f, Aeq, beq, Ain, bin, xWrapper, false);
         wrapperSolverTimer.stopMeasurement();

         DenseMatrix64F bEqualityVerify = new DenseMatrix64F(numberOfEqualityConstraints, 1);
         CommonOps.mult(Aeq, x, bEqualityVerify);

         // Verify Ax=b Equality constraints hold:
         JUnitTools.assertMatrixEquals(bEqualityVerify, beq, 1e-7);

         // Verify Ax<b Inequality constraints hold:
         DenseMatrix64F bInequalityVerify = new DenseMatrix64F(numberOfInequalityConstraints, 1);
         CommonOps.mult(Ain, x, bInequalityVerify);

         for (int j=0; j<bInequalityVerify.getNumRows(); j++)
         {
            Assert.assertTrue(bInequalityVerify.get(j, 0) < beq.get(j, 0));
         }

         // Verify solution is as expected
         Assert.assertArrayEquals("repeat = " + repeat, x.getData(), xWrapper.getData(), 1e-10);
      }

      PrintTools.info("Wrapper solve time : " + wrapperSolverTimer.getAverageTime());
      PrintTools.info("Java solve time : " + javaSolverTimer.getAverageTime());
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testAgainstStandardQuadProg() throws NoConvergenceException
   {
      int numberOfInequalityConstraints = 1;
      int numberOfEqualityConstraints = 1;
      int numberOfVariables = 2;

      DenseMatrix64F Q = new DenseMatrix64F(numberOfVariables, numberOfVariables, true, 1, 0, 0, 1);
      DenseMatrix64F f = new DenseMatrix64F(numberOfVariables, 1, true, 1, 0);
      DenseMatrix64F Aeq = new DenseMatrix64F(numberOfEqualityConstraints, numberOfVariables, true, 1, 1);
      DenseMatrix64F beq = new DenseMatrix64F(numberOfEqualityConstraints, 1, true, 0);
      DenseMatrix64F Ain = new DenseMatrix64F(numberOfInequalityConstraints, numberOfVariables, true, 2, 1);
      DenseMatrix64F bin = new DenseMatrix64F(numberOfInequalityConstraints, 1, true, 0);

      JavaQuadProgSolver javaSolver = new JavaQuadProgSolver();
      QuadProgSolver solver = new QuadProgSolver();

      for (int repeat = 0; repeat < 10000; repeat++)
      {
         DenseMatrix64F x = new DenseMatrix64F(numberOfVariables, 1, true, -1, 1);
         javaSolver.solve(Q, f, Aeq, beq, Ain, bin, x, false);
         Assert.assertArrayEquals(x.getData(), new double[] { -0.5, 0.5 }, 1e-10);
      }

      numberOfInequalityConstraints = 1;
      numberOfEqualityConstraints = 2;
      numberOfVariables = 3;

      Q = new DenseMatrix64F(numberOfVariables, numberOfVariables, true, 1, 0, 1, 0, 1, 2, 1, 3, 7);
      f = new DenseMatrix64F(numberOfVariables, 1, true, 1, 0, 9);
      Aeq = new DenseMatrix64F(numberOfEqualityConstraints, numberOfVariables, true, 1, 1, 1, 2, 3, 4);
      beq = new DenseMatrix64F(numberOfEqualityConstraints, 1, true, 0, 7);
      Ain = new DenseMatrix64F(numberOfInequalityConstraints, numberOfVariables, true, 2, 1, 3);
      bin = new DenseMatrix64F(numberOfInequalityConstraints, 1, true, 0);

      for (int repeat = 0; repeat < 10000; repeat++)
      {
         DenseMatrix64F x = new DenseMatrix64F(numberOfVariables, 1, true, -1, 1, 3);
         DenseMatrix64F xWrapper = new DenseMatrix64F(numberOfVariables, 1, true, -1, 1, 3);

         javaSolver.solve(Q, f, Aeq, beq, Ain, bin, x, false);
         solver.solve(Q, f, Aeq, beq, Ain, bin, xWrapper, false);

         DenseMatrix64F bEqualityVerify = new DenseMatrix64F(numberOfEqualityConstraints, 1);
         CommonOps.mult(Aeq, x, bEqualityVerify);

         // Verify Ax=b Equality constraints hold:
         JUnitTools.assertMatrixEquals(bEqualityVerify, beq, 1e-7);

         // Verify Ax<b Inequality constraints hold:
         DenseMatrix64F bInequalityVerify = new DenseMatrix64F(numberOfInequalityConstraints, 1);
         CommonOps.mult(Ain, x, bInequalityVerify);

         for (int j=0; j<bInequalityVerify.getNumRows(); j++)
         {
            Assert.assertTrue(bInequalityVerify.get(j, 0) < beq.get(j, 0));
         }

         // Verify solution is as expected
         Assert.assertArrayEquals("repeat = " + repeat, x.getData(), xWrapper.getData(), 1e-10);
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testSolveProblemWithParallelConstraints() throws NoConvergenceException
   {
      // our simple active set solver can not solve this:
      // test problem: x <= -1 and x <= -2
      DenseMatrix64F Q = new DenseMatrix64F(1, 1);
      DenseMatrix64F Ain = new DenseMatrix64F(2, 1);
      DenseMatrix64F bin = new DenseMatrix64F(2, 1);
      DenseMatrix64F x = new DenseMatrix64F(1, 1);

      Q.set(0, 0, 1.0);
      Ain.set(0, 0, 1.0);
      Ain.set(1, 0, 1.0);
      bin.set(0, -1.0);
      bin.set(0, -2.0);

      DenseMatrix64F f = new DenseMatrix64F(1, 1);
      DenseMatrix64F Aeq = new DenseMatrix64F(0, 1);
      DenseMatrix64F beq = new DenseMatrix64F(0, 1);

      JavaQuadProgSolver solver = new JavaQuadProgSolver();

      PrintTools.info("Attempting to solve problem with: " + solver.getClass().getSimpleName());
      solver.solve(Q, f, Aeq, beq, Ain, bin, x, true);
      boolean correct = MathTools.epsilonEquals(-2.0, x.get(0), 10E-10);
      if (!correct)
         PrintTools.info("Failed. Result was " + x.get(0) + ", expected -2.0");
   }
}

