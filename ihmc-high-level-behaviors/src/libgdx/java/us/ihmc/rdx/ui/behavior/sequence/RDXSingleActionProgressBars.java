package us.ihmc.rdx.ui.behavior.sequence;

import imgui.extension.implot.flag.ImPlotFlags;
import imgui.internal.ImGui;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImPlotBasicDoublePlotLine;
import us.ihmc.rdx.imgui.ImPlotPlot;
import us.ihmc.rdx.ui.behavior.actions.RDXFootstepPlanAction;
import us.ihmc.rdx.ui.behavior.actions.RDXHandPoseAction;
import us.ihmc.rdx.ui.behavior.actions.RDXWalkAction;

public class RDXSingleActionProgressBars
{
   public static final float PROGRESS_BAR_HEIGHT = 18.0f;
   public static final float PLOT_HEIGHT = 40.0f;

   private RDXActionNode<?, ?> action;
   private final ImPlotPlot positionErrorPlot = new ImPlotPlot();
   private final ImPlotBasicDoublePlotLine currentPositionPlotLine = new ImPlotBasicDoublePlotLine();
   private final ImPlotPlot orientationErrorPlot = new ImPlotPlot();
   private final ImPlotBasicDoublePlotLine currentOrientationPlotLine = new ImPlotBasicDoublePlotLine();
   private final ImPlotPlot handForcePlot = new ImPlotPlot();
   private final ImPlotBasicDoublePlotLine handForcePlotLine = new ImPlotBasicDoublePlotLine();
   private double elapsedExecutionTime = -1.0;
   private boolean newlyExecuting = false;

   public RDXSingleActionProgressBars()
   {
      positionErrorPlot.setFlags(positionErrorPlot.getFlags() | ImPlotFlags.NoLegend);
      positionErrorPlot.getPlotLines().add(currentPositionPlotLine);
      positionErrorPlot.setCustomBeforePlotLogic(() -> currentPositionPlotLine.setLimitYMin(0.1));

      orientationErrorPlot.setFlags(orientationErrorPlot.getFlags() | ImPlotFlags.NoLegend);
      orientationErrorPlot.getPlotLines().add(currentOrientationPlotLine);
      orientationErrorPlot.setCustomBeforePlotLogic(() -> currentOrientationPlotLine.setLimitYMin(45.0));
   }

   public void update()
   {
      double newElapsedExecutionTime = action.getState().getElapsedExecutionTime();
      newlyExecuting = newElapsedExecutionTime < elapsedExecutionTime;
      elapsedExecutionTime = newElapsedExecutionTime;
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

      if (renderAsPlots)
      {
         if (newlyExecuting)
         {
            currentOrientationPlotLine.clear();
         }
         if (action.getState().getIsExecuting())
         {
            currentOrientationPlotLine.setDataColor(dataColor);
            currentOrientationPlotLine.addValue(Math.toDegrees(currentOrientationError));
         }
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

   // TODO: Put into RDXWalkAction
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
      }

      ImGui.progressBar((float) percentLeft, ImGui.getColumnWidth(), PROGRESS_BAR_HEIGHT, overlay);
   }

   // TODO: Put into RDXHandPoseAction
   public void renderHandForce(float dividedBarWidth, boolean renderAsPlots)
   {
      if (action instanceof RDXHandPoseAction handPoseAction)
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
