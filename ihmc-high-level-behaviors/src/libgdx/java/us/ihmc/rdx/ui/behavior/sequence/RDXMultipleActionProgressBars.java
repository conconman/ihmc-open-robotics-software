package us.ihmc.rdx.ui.behavior.sequence;

import imgui.internal.ImGui;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.actions.RDXFootstepPlanAction;
import us.ihmc.rdx.ui.behavior.actions.RDXHandPoseAction;
import us.ihmc.rdx.ui.behavior.actions.RDXWalkAction;
import us.ihmc.robotics.EuclidCoreMissingTools;

public class RDXMultipleActionProgressBars
{
   public static final float PROGRESS_BAR_HEIGHT = 18.0f;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RecyclingArrayList<RDXSingleActionProgressBars> actionProgressBars = new RecyclingArrayList<>(RDXSingleActionProgressBars::new);
   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
   private boolean renderAsPlots = false;

   public void render()
   {
      widgetAligner.text("Expected time remaining:");

      // We compute the bar width to show them all together,
      // but we need to account for the spacing between them.
      float barWidthToSubtract = 0.0f;
      if (actionProgressBars.size() > 1)
      {
         int barsPastOne = actionProgressBars.size() - 1;
         float totalInnerSpacing = ImGui.getStyle().getItemSpacingX() * barsPastOne;
         barWidthToSubtract = totalInnerSpacing / actionProgressBars.size();
      }
      float dividedBarWidth = ImGui.getColumnWidth() / actionProgressBars.size() - barWidthToSubtract;

      for (RDXSingleActionProgressBars actionProgressBar : actionProgressBars)
      {
         actionProgressBar.update();
      }

      handleRenderingBlankBar();
      for (int i = 0; i < actionProgressBars.size(); i++)
      {
         double elapsedTime = actionProgressBars.get(i).getAction().getState().getElapsedExecutionTime();
         double nominalDuration = actionProgressBars.get(i).getAction().getState().getNominalExecutionDuration();
         double percentComplete = elapsedTime / nominalDuration;
         double percentLeft = 1.0 - percentComplete;
         ImGui.progressBar((float) percentLeft, dividedBarWidth, PROGRESS_BAR_HEIGHT, "%.2f / %.2f".formatted(elapsedTime, nominalDuration));
         sameLineExceptLast(i);
      }
      ImGui.spacing();
      widgetAligner.text("Position error (m):");
      handleRenderingBlankBar();
      for (int i = 0; i < actionProgressBars.size(); i++)
      {
         actionProgressBars.get(i).renderPositionError(dividedBarWidth, renderAsPlots);
         sameLineExceptLast(i);
      }
      ImGui.spacing();
      widgetAligner.text("Orientation error (%s):".formatted(EuclidCoreMissingTools.DEGREE_SYMBOL));
      handleRenderingBlankBar();
      for (int i = 0; i < actionProgressBars.size(); i++)
      {
         double currentOrientationError = actionProgressBars.get(i).getAction().getState().getCurrentOrientationDistanceToGoal();
         double startOrientationError = actionProgressBars.get(i).getAction().getState().getStartOrientationDistanceToGoal();
         double orientationTolerance = actionProgressBars.get(i).getAction().getState().getOrientationDistanceToGoalTolerance();
         double barEndValue = Math.max(Math.min(startOrientationError, currentOrientationError), 2.0 * orientationTolerance);
         double toleranceMarkPercent = orientationTolerance / barEndValue;
         int barColor = currentOrientationError < orientationTolerance ? ImGuiTools.GREEN : ImGuiTools.RED;
         double percentLeft = currentOrientationError / barEndValue;
         ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT,
                                      dividedBarWidth,
                                      barColor,
                                      percentLeft,
                                      toleranceMarkPercent,
                                      "%.2f / %.2f".formatted(Math.toDegrees(currentOrientationError), Math.toDegrees(startOrientationError)));
         sameLineExceptLast(i);
      }
      ImGui.spacing();
      widgetAligner.text("Footstep completion:");
      handleRenderingBlankBar();
      for (int i = 0; i < actionProgressBars.size(); i++)
      {
         int incompleteFootsteps = 0;
         int totalFootsteps = 0;
         double percentLeft = Double.NaN;
         String overlay = "N/A";
         if (actionProgressBars.get(i).getAction() instanceof RDXWalkAction walkAction)
         {
            incompleteFootsteps = walkAction.getState().getNumberOfIncompleteFootsteps();
            totalFootsteps = walkAction.getState().getTotalNumberOfFootsteps();
         }
         if (actionProgressBars.get(i).getAction() instanceof RDXFootstepPlanAction footstepPlanAction)
         {
            incompleteFootsteps = footstepPlanAction.getState().getNumberOfIncompleteFootsteps();
            totalFootsteps = footstepPlanAction.getState().getTotalNumberOfFootsteps();
         }
         if (totalFootsteps > 0)
         {
            percentLeft = incompleteFootsteps / (double) totalFootsteps;
            overlay = "%d / %d".formatted(incompleteFootsteps, totalFootsteps);
         }

         ImGui.progressBar((float) percentLeft, ImGui.getColumnWidth(), PROGRESS_BAR_HEIGHT, overlay);
         sameLineExceptLast(i);
      }
      ImGui.spacing();
      widgetAligner.text("Hand wrench linear (N?):");
      handleRenderingBlankBar();
      for (int i = 0; i < actionProgressBars.size(); i++)
      {
         if (actionProgressBars.get(i).getAction() instanceof RDXHandPoseAction handPoseAction)
         {
            double limit = 20.0;
            double force = handPoseAction.getState().getHandWrenchMagnitudeLinear();
            int barColor = force < limit ? ImGuiTools.GREEN : ImGuiTools.RED;
            ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT, dividedBarWidth, barColor, force / limit, 0.5, "%.2f".formatted(force));
         }
         else
         {
            ImGui.progressBar(Float.NaN, ImGui.getColumnWidth(), PROGRESS_BAR_HEIGHT, "N/A");
         }
         sameLineExceptLast(i);
      }
      ImGui.spacing();
   }

   private void sameLineExceptLast(int i)
   {
      if (i < actionProgressBars.size() - 1)
         ImGui.sameLine();
   }

   private void handleRenderingBlankBar()
   {
      if (actionProgressBars.isEmpty())
         ImGui.progressBar(Float.NaN, ImGui.getColumnWidth(), PROGRESS_BAR_HEIGHT, "");
   }

   public RecyclingArrayList<RDXSingleActionProgressBars> getActionProgressBars()
   {
      return actionProgressBars;
   }

   public boolean getRenderAsPlots()
   {
      return renderAsPlots;
   }

   public void setRenderAsPlots(boolean renderAsPlots)
   {
      this.renderAsPlots = renderAsPlots;
   }
}
