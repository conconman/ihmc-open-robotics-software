package us.ihmc.rdx.ui.behavior.sequence;

import imgui.internal.ImGui;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.ui.behavior.actions.RDXFootstepPlanAction;
import us.ihmc.rdx.ui.behavior.actions.RDXWalkAction;
import us.ihmc.robotics.EuclidCoreMissingTools;

import java.util.ArrayList;

public class RDXActionProgressWidgetsManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
   private final ArrayList<RDXActionNode<?, ?>> actionNodesToRender = new ArrayList<>();
   private boolean renderAsPlots = false;
   private int emptyPlotIndex = 0;

   public void render()
   {
      emptyPlotIndex = 0;

      for (RDXActionNode<?, ?> action : actionNodesToRender)
      {
         action.getProgressWidgets().update();
      }

      widgetAligner.text("Expected time remaining:");
      float dividedBarWidth = computeDividedBarWidth(); // Must be computed after above text
      handleRenderingBlankBar(false);
      for (int i = 0; i < actionNodesToRender.size(); i++)
      {
         actionNodesToRender.get(i).getProgressWidgets().renderElapsedTimeBar(dividedBarWidth);
         sameLineExceptLast(i);
      }
      ImGui.spacing();

      widgetAligner.text("Position error (m):");
      handleRenderingBlankBar(true);
      for (int i = 0; i < actionNodesToRender.size(); i++)
      {
         if (actionNodesToRender.get(i) instanceof RDXWalkAction || actionNodesToRender.get(i) instanceof RDXFootstepPlanAction)
            actionNodesToRender.get(i).getProgressWidgets().renderFootPositions(dividedBarWidth, renderAsPlots);
         else
            actionNodesToRender.get(i).getProgressWidgets().renderPositionError(dividedBarWidth, renderAsPlots);
         sameLineExceptLast(i);
      }
      ImGui.spacing();

      widgetAligner.text("Orientation error (%s):".formatted(EuclidCoreMissingTools.DEGREE_SYMBOL));
      handleRenderingBlankBar(true);
      for (int i = 0; i < actionNodesToRender.size(); i++)
      {
         actionNodesToRender.get(i).getProgressWidgets().renderOrientationError(dividedBarWidth, renderAsPlots);
         sameLineExceptLast(i);
      }
      ImGui.spacing();

      widgetAligner.text("Hand force (N):");
      handleRenderingBlankBar(true);
      for (int i = 0; i < actionNodesToRender.size(); i++)
      {
         actionNodesToRender.get(i).getProgressWidgets().renderHandForce(dividedBarWidth, renderAsPlots);
         sameLineExceptLast(i);
      }
      ImGui.spacing();

      widgetAligner.text("Hand torque (Nm):");
      handleRenderingBlankBar(true);
      for (int i = 0; i < actionNodesToRender.size(); i++)
      {
         actionNodesToRender.get(i).getProgressWidgets().renderHandTorque(dividedBarWidth, renderAsPlots);
         sameLineExceptLast(i);
      }
      ImGui.spacing();

      boolean containsFootstepAction = false;
      for (RDXActionNode<?, ?> action : actionNodesToRender)
         if (action instanceof RDXWalkAction || action instanceof RDXFootstepPlanAction)
            containsFootstepAction = true;
      if (containsFootstepAction)
      {
         widgetAligner.text("Footstep completion:");
         handleRenderingBlankBar(true);
         for (int i = 0; i < actionNodesToRender.size(); i++)
         {
            actionNodesToRender.get(i).getProgressWidgets().renderFootstepCompletion(dividedBarWidth, renderAsPlots);
            sameLineExceptLast(i);
         }
         ImGui.spacing();
      }
   }

   private float computeDividedBarWidth()
   {
      // We compute the bar width to show them all together,
      // but we need to account for the spacing between them.
      float barWidthToSubtract = 0.0f;
      if (actionNodesToRender.size() > 1)
      {
         int barsPastOne = actionNodesToRender.size() - 1;
         float totalInnerSpacing = ImGui.getStyle().getItemSpacingX() * barsPastOne;
         barWidthToSubtract = totalInnerSpacing / actionNodesToRender.size();
      }
      return ImGui.getColumnWidth() / actionNodesToRender.size() - barWidthToSubtract;
   }

   private void sameLineExceptLast(int i)
   {
      if (i < actionNodesToRender.size() - 1)
         ImGui.sameLine();
   }

   private void handleRenderingBlankBar(boolean supportsPlots)
   {
      if (actionNodesToRender.isEmpty())
      {
         RDXActionProgressWidgets.renderBlankProgress(labels.get("Empty Plot", emptyPlotIndex++), ImGui.getColumnWidth(), renderAsPlots, supportsPlots);
      }
   }

   public boolean getRenderAsPlots()
   {
      return renderAsPlots;
   }

   public void setRenderAsPlots(boolean renderAsPlots)
   {
      this.renderAsPlots = renderAsPlots;
   }

   public ArrayList<RDXActionNode<?, ?>> getActionNodesToRender()
   {
      return actionNodesToRender;
   }
}
