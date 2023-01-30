package us.ihmc.valkyrie;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

public class FootstepPlanGenerator
{
   public FootstepPlanGenerator()
   {
      ArrayList<FootstepPlan> referenced_output_plans = new ArrayList<>();

      FootstepPlanningModule planningModule = new FootstepPlanningModule("planningModule");
      FootstepPlannerLogger logger = new FootstepPlannerLogger(planningModule);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setStartFootPose(RobotSide.LEFT, new Pose3D(0.0, 0.1, 0.0, 0.0, 0.0, 0.0));
      request.setStartFootPose(RobotSide.RIGHT, new Pose3D(0.0, -0.1, 0.0, 0.0, 0.0, 0.0));
      request.setGoalFootPose(RobotSide.LEFT, new Pose3D(0.6, 0.1, 0.0, 0.0, 0.0, 0.0));
      request.setGoalFootPose(RobotSide.RIGHT, new Pose3D(0.6, -0.1, 0.0, 0.0, 0.0, 0.0));
      request.setRequestedInitialStanceSide(RobotSide.LEFT);

      // Case: NOMINAL plan
      FootstepPlannerOutput nominal_output = planningModule.handleRequest(request);
      planningModule.handleRequest(request);
      logger.logSession();

      // Case: REFERENCE plan A
      FootstepPlan reference_A = new FootstepPlan();
      reference_A.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(0.3, -0.2, 0.0, 0.0, 0.0, 0.0)));
      reference_A.addFootstep(RobotSide.LEFT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(0.6, 0.2, 0.0, 0.0, 0.0, 0.0)));
      request.setReferencePlan(reference_A);

      // ref A alpha 1
      FootstepPlan ref_A_alpha_full_output = planAndLog(1.0, planningModule, request, logger);
      referenced_output_plans.add(ref_A_alpha_full_output);
      // ref A alpha 0.5
      FootstepPlan ref_A_alpha_half_output = planAndLog(0.5, planningModule, request, logger);
      referenced_output_plans.add(ref_A_alpha_half_output);
      // ref A alpha 0.0
      FootstepPlan ref_A_alpha_zero_output = planAndLog(0.0, planningModule, request, logger);
      referenced_output_plans.add(ref_A_alpha_zero_output);

      // Case: REFERENCE plan B
      FootstepPlan reference_B = new FootstepPlan();
      reference_B.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(0.25, -0.05, 0.0, 0.0, 0.0, 0.0)));
      reference_B.addFootstep(RobotSide.LEFT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(0.25, 0.25, 0.0, 0.0, 0.0, 0.0)));
      request.setReferencePlan(reference_B);

      // ref B alpha 1
      FootstepPlan ref_B_alpha_full_output = planAndLog(1.0, planningModule, request, logger);
      referenced_output_plans.add(ref_B_alpha_full_output);
      // ref B alpha 0.5
      FootstepPlan ref_B_alpha_half_output = planAndLog(0.5, planningModule, request, logger);
      referenced_output_plans.add(ref_B_alpha_half_output);
      // ref B alpha 0.0
      FootstepPlan ref_B_alpha_zero_output = planAndLog(0.0, planningModule, request, logger);
      referenced_output_plans.add(ref_B_alpha_zero_output);

      // Plan without a reference:
      // RIGHT ( 0.300, -0.100,  0.000 )
      // LEFT  ( 0.600,  0.100,  0.000 )
      // RIGHT ( 0.600, -0.100,  0.000 )

      System.out.println("NOMINAL output");
      printOutput(nominal_output);
      System.out.println("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      System.out.println("REF outputs with alphas 1.0, 0.5 ,0.0");
      for (FootstepPlan plan : referenced_output_plans)
      {
         printPlan(plan);
         System.out.println("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      }
   }

   private void printPlan(FootstepPlan plan)
   {
      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         PlannedFootstep step = plan.getFootstep(i);
         System.out.println(step.getRobotSide() + ", \n" + step.getFootstepPose());
      }
   }

   private void printOutput(FootstepPlannerOutput output)
   {
      FootstepPlan outputPlan = output.getFootstepPlan();
      for (int i = 0; i < outputPlan.getNumberOfSteps(); i++)
      {
         PlannedFootstep step = outputPlan.getFootstep(i);
         System.out.println(step.getRobotSide() + ", \n" + step.getFootstepPose());
      }
   }

   private FootstepPlan planAndLog(double alpha, FootstepPlanningModule planningModule, FootstepPlannerRequest request, FootstepPlannerLogger logger)
   {
      planningModule.getAStarFootstepPlanner().getReferenceBasedIdealStepCalculator().setReferenceAlpha(alpha);
      FootstepPlannerOutput output = planningModule.handleRequest(request);
      FootstepPlan plan = new FootstepPlan(output.getFootstepPlan());

      logger.logSession();
      return plan;
   }

   public static void main(String[] args)
   {
      new FootstepPlanGenerator();
   }
}
