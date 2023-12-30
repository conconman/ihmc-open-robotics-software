package us.ihmc.rdx.ui.behavior.sequence;

import imgui.internal.ImGui;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.rdx.imgui.*;
import us.ihmc.robotics.EuclidCoreMissingTools;

public class RDXMultipleActionProgressBars
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RecyclingArrayList<RDXSingleActionProgressBars> actionProgressBars = new RecyclingArrayList<>(RDXSingleActionProgressBars::new);
   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
   private boolean renderAsPlots = false;
   private int emptyPlotIndex = 0;

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

      emptyPlotIndex = 0;
      handleRenderingBlankBar(false);
      for (int i = 0; i < actionProgressBars.size(); i++)
      {
         actionProgressBars.get(i).renderElapsedTimeBar(dividedBarWidth);
         sameLineExceptLast(i);
      }
      ImGui.spacing();
      widgetAligner.text("Position error (m):");
      handleRenderingBlankBar(true);
      for (int i = 0; i < actionProgressBars.size(); i++)
      {
         actionProgressBars.get(i).renderPositionError(dividedBarWidth, renderAsPlots);
         sameLineExceptLast(i);
      }
      ImGui.spacing();
      widgetAligner.text("Orientation error (%s):".formatted(EuclidCoreMissingTools.DEGREE_SYMBOL));
      handleRenderingBlankBar(true);
      for (int i = 0; i < actionProgressBars.size(); i++)
      {
         actionProgressBars.get(i).renderOrientationError(dividedBarWidth, renderAsPlots);
         sameLineExceptLast(i);
      }
      ImGui.spacing();
      widgetAligner.text("Footstep completion:");
      handleRenderingBlankBar(false);
      for (int i = 0; i < actionProgressBars.size(); i++)
      {
         actionProgressBars.get(i).renderFootstepCompletion(dividedBarWidth, renderAsPlots);
         sameLineExceptLast(i);
      }
      ImGui.spacing();
      widgetAligner.text("Hand wrench linear (N?):");
      handleRenderingBlankBar(true);
      for (int i = 0; i < actionProgressBars.size(); i++)
      {
         actionProgressBars.get(i).renderHandForce(dividedBarWidth, renderAsPlots);
         sameLineExceptLast(i);
      }
      ImGui.spacing();
   }

   private void sameLineExceptLast(int i)
   {
      if (i < actionProgressBars.size() - 1)
         ImGui.sameLine();
   }

   private void handleRenderingBlankBar(boolean supportsPlots)
   {
      if (actionProgressBars.isEmpty())
      {
         if (renderAsPlots && supportsPlots)
         {
            ImPlotTools.renderEmptyPlotArea(labels.get("Empty Plot", emptyPlotIndex++), ImGui.getColumnWidth(), RDXSingleActionProgressBars.PLOT_HEIGHT);
         }
         else
         {
            ImGui.progressBar(Float.NaN, ImGui.getColumnWidth(), RDXSingleActionProgressBars.PROGRESS_BAR_HEIGHT, "");
         }
      }
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
