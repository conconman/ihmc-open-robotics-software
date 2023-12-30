package us.ihmc.rdx.ui.behavior.sequence;

import imgui.extension.implot.flag.ImPlotFlags;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImPlotBasicDoublePlotLine;
import us.ihmc.rdx.imgui.ImPlotPlot;

public class RDXSingleActionProgressBars
{
   public static final float PROGRESS_BAR_HEIGHT = 18.0f;
   public static final float PLOT_HEIGHT = 40.0f;

   private RDXActionNode<?, ?> action;
   private final ImPlotPlot positionErrorPlot;
   private final ImPlotBasicDoublePlotLine currentPositionPlotLine = new ImPlotBasicDoublePlotLine();
   private double elapsedExecutionTime = -1.0;
   private boolean newlyExecuting = false;

   public RDXSingleActionProgressBars()
   {
      positionErrorPlot = new ImPlotPlot();
      currentPositionPlotLine.setLegendLabel("Current position error (m)");
      positionErrorPlot.getPlotLines().add(currentPositionPlotLine);
      positionErrorPlot.setFlags(positionErrorPlot.getFlags() | ImPlotFlags.NoLegend);
      positionErrorPlot.setCustomBeforePlotLogic(() -> currentPositionPlotLine.setLimitYMin(0.1));

   }

   public void update()
   {
      double newElapsedExecutionTime = action.getState().getElapsedExecutionTime();
      newlyExecuting = newElapsedExecutionTime < elapsedExecutionTime;
      elapsedExecutionTime = newElapsedExecutionTime;
   }

   public void renderPositionError(float dividedBarWidth, boolean renderAsPlots)
   {
      double currentPositionError = action.getState().getCurrentPositionDistanceToGoal();
      double startPositionError = action.getState().getStartPositionDistanceToGoal();
      double positionTolerance = action.getState().getPositionDistanceToGoalTolerance();
      double barEndValue = Math.max(Math.min(startPositionError, currentPositionError), 2.0 * positionTolerance);
      double toleranceMarkPercent = positionTolerance / barEndValue;
      int dataColor = currentPositionError < positionTolerance ? ImGuiTools.GREEN : ImGuiTools.RED;
      double percentLeft = currentPositionError / barEndValue;

      if (renderAsPlots)
      {
         if (newlyExecuting)
         {
            currentPositionPlotLine.clear();
         }
         if (action.getState().getIsExecuting())
         {
            currentPositionPlotLine.setDataColor(dataColor);
            currentPositionPlotLine.addValue(currentPositionError);
         }
         positionErrorPlot.render(dividedBarWidth, PLOT_HEIGHT);
      }
      else
      {
         ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT,
                                      dividedBarWidth,
                                      dataColor,
                                      percentLeft,
                                      toleranceMarkPercent,
                                      "%.2f / %.2f".formatted(currentPositionError, startPositionError));
      }
   }

   public void setAction(RDXActionNode<?, ?> action)
   {
      this.action = action;
   }

   public RDXActionNode<?, ?> getAction()
   {
      return action;
   }
}
