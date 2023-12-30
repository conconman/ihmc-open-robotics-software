package us.ihmc.rdx.ui.behavior.sequence;

import imgui.extension.implot.flag.ImPlotFlags;
import imgui.internal.ImGui;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.ui.behavior.actions.RDXFootstepPlanAction;
import us.ihmc.rdx.ui.behavior.actions.RDXHandPoseAction;
import us.ihmc.rdx.ui.behavior.actions.RDXWalkAction;

public class RDXActionProgressWidgets
{
   public static final float PROGRESS_BAR_HEIGHT = 18.0f;
   public static final float PLOT_HEIGHT = 40.0f;

   private final RDXActionNode<?, ?> action;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImPlotPlot positionErrorPlot = new ImPlotPlot();
   private final ImPlotBasicDoublePlotLine currentPositionPlotLine = new ImPlotBasicDoublePlotLine();
   private final ImPlotPlot orientationErrorPlot = new ImPlotPlot();
   private final ImPlotBasicDoublePlotLine currentOrientationPlotLine = new ImPlotBasicDoublePlotLine();
   private final ImPlotPlot footstepsRemainingPlot = new ImPlotPlot();
   private final ImPlotBasicDoublePlotLine footstepsRemainingPlotLine = new ImPlotBasicDoublePlotLine();
   private final ImPlotPlot handForcePlot = new ImPlotPlot();
   private final ImPlotBasicDoublePlotLine handForcePlotLine = new ImPlotBasicDoublePlotLine();
   private double elapsedExecutionTime = -1.0;
   private boolean newlyExecuting = false;

   public RDXActionProgressWidgets(RDXActionNode<?, ?> action)
   {
      this.action = action;

      setupPlot(positionErrorPlot, currentPositionPlotLine, 0.1);
      setupPlot(orientationErrorPlot, currentOrientationPlotLine, 45.0);
      setupPlot(footstepsRemainingPlot, footstepsRemainingPlotLine, 1.0);
      setupPlot(handForcePlot, handForcePlotLine, 50.0);
   }

   private void setupPlot(ImPlotPlot plot, ImPlotBasicDoublePlotLine plotLine, double limitYMin)
   {
      plot.setFlag(ImPlotFlags.NoLegend);
      plot.getPlotLines().add(plotLine);
      plot.setCustomBeforePlotLogic(() -> plotLine.setLimitYMin(limitYMin));
   }

   public void update()
   {
      double newElapsedExecutionTime = action.getState().getElapsedExecutionTime();
      newlyExecuting = newElapsedExecutionTime < elapsedExecutionTime;
      elapsedExecutionTime = newElapsedExecutionTime;

      if (newlyExecuting)
      {
         currentPositionPlotLine.clear();
         currentOrientationPlotLine.clear();
         footstepsRemainingPlotLine.clear();
         handForcePlotLine.clear();
      }
   }

   public void renderElapsedTimeBar(float dividedBarWidth)
   {
      double elapsedTime = action.getState().getElapsedExecutionTime();
      double nominalDuration = action.getState().getNominalExecutionDuration();
      double percentComplete = elapsedTime / nominalDuration;
      double percentLeft = 1.0 - percentComplete;
      ImGui.progressBar((float) percentLeft, dividedBarWidth, PROGRESS_BAR_HEIGHT, "%.2f / %.2f".formatted(elapsedTime, nominalDuration));
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

      if (action.getState().getIsExecuting())
      {
         currentPositionPlotLine.setDataColor(dataColor);
         currentPositionPlotLine.addValue(currentPositionError);
      }
      if (renderAsPlots)
      {
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

   public void renderOrientationError(float dividedBarWidth, boolean renderAsPlots)
   {
      orientationErrorPlot.setCustomBeforePlotLogic(() -> currentOrientationPlotLine.setLimitYMin(45.0));
      double currentOrientationError = action.getState().getCurrentOrientationDistanceToGoal();
      double startOrientationError = action.getState().getStartOrientationDistanceToGoal();
      double orientationTolerance = action.getState().getOrientationDistanceToGoalTolerance();
      double barEndValue = Math.max(Math.min(startOrientationError, currentOrientationError), 2.0 * orientationTolerance);
      double toleranceMarkPercent = orientationTolerance / barEndValue;
      int dataColor = currentOrientationError < orientationTolerance ? ImGuiTools.GREEN : ImGuiTools.RED;
      double percentLeft = currentOrientationError / barEndValue;

      if (action.getState().getIsExecuting())
      {
         currentOrientationPlotLine.setDataColor(dataColor);
         currentOrientationPlotLine.addValue(Math.toDegrees(currentOrientationError));
      }
      if (renderAsPlots)
      {
         orientationErrorPlot.render(dividedBarWidth, PLOT_HEIGHT);
      }
      else
      {
         ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT,
                                      dividedBarWidth,
                                      dataColor,
                                      percentLeft,
                                      toleranceMarkPercent,
                                      "%.2f / %.2f".formatted(Math.toDegrees(currentOrientationError), Math.toDegrees(startOrientationError)));
      }
   }

   public void renderHandForce(float dividedBarWidth, boolean renderAsPlots)
   {
      if (action instanceof RDXHandPoseAction handPoseAction)
      {
         double limit = 20.0;
         double force = handPoseAction.getState().getHandWrenchMagnitudeLinear();
         int dataColor = force < limit ? ImGuiTools.GREEN : ImGuiTools.RED;

         if (action.getState().getIsExecuting())
         {
            handForcePlotLine.setDataColor(dataColor);
            handForcePlotLine.addValue(force);
         }
         if (renderAsPlots)
         {
            handForcePlot.render(dividedBarWidth, PLOT_HEIGHT);
         }
         else
         {
            ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT, dividedBarWidth, dataColor, force / limit, 0.5, "%.2f".formatted(force));
         }
      }
      else
      {
         renderBlankProgress(labels.get("Empty Hand Force"), dividedBarWidth, renderAsPlots, true);
      }
   }

   public void renderFootstepCompletion(float dividedBarWidth, boolean renderAsPlots)
   {
      int incompleteFootsteps = 0;
      int totalFootsteps = 0;
      double percentLeft = Double.NaN;
      String overlay = "N/A";
      if (action instanceof RDXWalkAction walkAction)
      {
         incompleteFootsteps = walkAction.getState().getNumberOfIncompleteFootsteps();
         totalFootsteps = walkAction.getState().getTotalNumberOfFootsteps();
      }
      if (action instanceof RDXFootstepPlanAction footstepPlanAction)
      {
         incompleteFootsteps = footstepPlanAction.getState().getNumberOfIncompleteFootsteps();
         totalFootsteps = footstepPlanAction.getState().getTotalNumberOfFootsteps();
      }
      if (totalFootsteps > 0)
      {
         percentLeft = incompleteFootsteps / (double) totalFootsteps;
         overlay = "%d / %d".formatted(incompleteFootsteps, totalFootsteps);

         if (action.getState().getIsExecuting())
         {
            footstepsRemainingPlotLine.addValue(incompleteFootsteps);
         }
         if (renderAsPlots)
         {
            footstepsRemainingPlot.render(dividedBarWidth, PLOT_HEIGHT);
         }
         else
         {
            ImGui.progressBar((float) percentLeft, dividedBarWidth, PROGRESS_BAR_HEIGHT, overlay);
         }
      }
      else
      {
         renderBlankProgress(labels.get("Empty Footsteps"), dividedBarWidth, renderAsPlots, true);
      }
   }

   public static void renderBlankProgress(String emptyPlotLabel, float width, boolean renderAsPlots, boolean supportsPlots)
   {
      if (renderAsPlots && supportsPlots)
      {
         ImPlotTools.renderEmptyPlotArea(emptyPlotLabel, width, RDXActionProgressWidgets.PLOT_HEIGHT);
      }
      else
      {
         ImGui.progressBar(Float.NaN, width, RDXActionProgressWidgets.PROGRESS_BAR_HEIGHT, "");
      }
   }
}
