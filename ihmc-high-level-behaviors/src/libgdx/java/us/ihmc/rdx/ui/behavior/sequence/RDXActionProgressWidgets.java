package us.ihmc.rdx.ui.behavior.sequence;

import imgui.extension.implot.flag.ImPlotFlags;
import imgui.internal.ImGui;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionStateBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.ui.behavior.actions.RDXFootstepPlanAction;
import us.ihmc.rdx.ui.behavior.actions.RDXHandPoseAction;
import us.ihmc.rdx.ui.behavior.actions.RDXWalkAction;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoRegistry;

public class RDXActionProgressWidgets
{
   public static final float PROGRESS_BAR_HEIGHT = 18.0f;
   public static final float PLOT_HEIGHT = 40.0f;

   private final RDXActionNode<?, ?> action;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImPlotPlot positionErrorPlot = new ImPlotPlot();
   private final ImPlotBasicDoublePlotLine currentPositionErrorPlotLine = new ImPlotBasicDoublePlotLine();
   private final ImPlotBasicDoublePlotLine desiredPositionErrorPlotLine = new ImPlotBasicDoublePlotLine();
   private final ImPlotPlot orientationErrorPlot = new ImPlotPlot();
   private final ImPlotBasicDoublePlotLine currentOrientationPlotLine = new ImPlotBasicDoublePlotLine();
   private final ImPlotBasicDoublePlotLine desiredOrientationPlotLine = new ImPlotBasicDoublePlotLine();
   private final ImPlotPlot footstepsRemainingPlot = new ImPlotPlot();
   private final ImPlotBasicDoublePlotLine footstepsRemainingPlotLine = new ImPlotBasicDoublePlotLine();
   private final ImPlotPlot handForcePlot = new ImPlotPlot();
   private final ImPlotBasicDoublePlotLine handForcePlotLine = new ImPlotBasicDoublePlotLine();
   private double elapsedExecutionTime = -1.0;
   private boolean newlyExecuting = false;
   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectoryGenerator
         = new MultipleWaypointsPositionTrajectoryGenerator("Position", 500, ReferenceFrame.getWorldFrame(), new YoRegistry("DummyParent"));
   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryGenerator
         = new MultipleWaypointsOrientationTrajectoryGenerator("Orientation", 500, ReferenceFrame.getWorldFrame(), new YoRegistry("DummyParent"));

   public RDXActionProgressWidgets(RDXActionNode<?, ?> action)
   {
      this.action = action;

      setupPlot(positionErrorPlot, 0.1, currentPositionErrorPlotLine, desiredPositionErrorPlotLine);
      setupPlot(orientationErrorPlot, 45.0, currentOrientationPlotLine, desiredOrientationPlotLine);
      setupPlot(footstepsRemainingPlot, 1.0, footstepsRemainingPlotLine);
      setupPlot(handForcePlot, 50.0, handForcePlotLine);
   }

   private void setupPlot(ImPlotPlot plot, double limitYMin, ImPlotBasicDoublePlotLine... plotLines)
   {
      plot.setFlag(ImPlotFlags.NoLegend);
      for (ImPlotBasicDoublePlotLine plotLine : plotLines)
      {
         plot.getPlotLines().add(plotLine);
      }
      plot.setCustomBeforePlotLogic(() ->
      {
         for (ImPlotBasicDoublePlotLine plotLine : plotLines)
         {
            plotLine.setLimitYMin(limitYMin);
         }
      });
   }

   public void update()
   {
      double newElapsedExecutionTime = action.getState().getElapsedExecutionTime();
      newlyExecuting = newElapsedExecutionTime < elapsedExecutionTime;
      elapsedExecutionTime = newElapsedExecutionTime;

      if (newlyExecuting)
      {
         currentPositionErrorPlotLine.clear();
         desiredPositionErrorPlotLine.clear();
         currentOrientationPlotLine.clear();
         desiredOrientationPlotLine.clear();
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
      if (!action.getState().getDesiredTrajectory().isEmpty())
      {
         positionTrajectoryGenerator.clear();
         for (int i = 0; i < action.getState().getDesiredTrajectory().getSize(); i++)
         {
            positionTrajectoryGenerator.appendWaypoint(action.getState().getDesiredTrajectory().getValueReadOnly(i));
         }
         positionTrajectoryGenerator.initialize();
         positionTrajectoryGenerator.compute(action.getState().getElapsedExecutionTime());

         Point3DReadOnly initialPosition = action.getState().getDesiredTrajectory().getFirstValueReadOnly().getPosition();
         Point3DReadOnly endPosition = action.getState().getDesiredTrajectory().getLastValueReadOnly().getPosition();
         Point3DReadOnly currentPosition = action.getState().getCurrentPose().getValueReadOnly().getPosition();
         Point3DReadOnly desiredPosition = positionTrajectoryGenerator.getPosition();

         double initialToEnd = initialPosition.differenceNorm(endPosition);
         double currentToEnd = currentPosition.differenceNorm(endPosition);
         double desiredToEnd = desiredPosition.differenceNorm(endPosition);
         double tolerance = action.getState().getPositionDistanceToGoalTolerance();
         double error = Math.abs(currentToEnd - desiredToEnd);
         int dataColor = error < tolerance ? ImGuiTools.GREEN : ImGuiTools.RED;

         if (action.getState().getIsExecuting())
         {
            currentPositionErrorPlotLine.setDataColor(dataColor);
            currentPositionErrorPlotLine.addValue(currentToEnd);
            desiredPositionErrorPlotLine.setDataColor(ImGuiTools.GRAY);
            desiredPositionErrorPlotLine.addValue(desiredToEnd);
         }
         if (renderAsPlots)
         {
            positionErrorPlot.render(dividedBarWidth, PLOT_HEIGHT);
         }
         else
         {
            double barEndValue = Math.max(Math.min(initialToEnd, currentToEnd), 2.0 * tolerance);
            double toleranceMarkPercent = tolerance / barEndValue;
            double percentLeft = currentToEnd / barEndValue;
            ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT,
                                         dividedBarWidth,
                                         dataColor,
                                         percentLeft,
                                         toleranceMarkPercent,
                                         "%.2f / %.2f".formatted(currentToEnd, initialToEnd));
         }
      }
      else
      {
         renderBlankProgress("Empty Position Error", dividedBarWidth, renderAsPlots, true);
      }
   }

   public void renderOrientationError(float dividedBarWidth, boolean renderAsPlots)
   {
      if (!action.getState().getDesiredTrajectory().isEmpty())
      {
         orientationTrajectoryGenerator.clear();
         for (int i = 0; i < action.getState().getDesiredTrajectory().getSize(); i++)
         {
            orientationTrajectoryGenerator.appendWaypoint(action.getState().getDesiredTrajectory().getValueReadOnly(i));
         }
         orientationTrajectoryGenerator.initialize();
         orientationTrajectoryGenerator.compute(action.getState().getElapsedExecutionTime());

         QuaternionReadOnly initialOrientation = action.getState().getDesiredTrajectory().getFirstValueReadOnly().getOrientation();
         QuaternionReadOnly endOrientation = action.getState().getDesiredTrajectory().getLastValueReadOnly().getOrientation();
         QuaternionReadOnly currentOrientation = action.getState().getCurrentPose().getValueReadOnly().getOrientation();
         QuaternionReadOnly desiredOrientation = orientationTrajectoryGenerator.getOrientation();

         double initialToEnd = initialOrientation.distance(endOrientation, true);
         double currentToEnd = currentOrientation.distance(endOrientation, true);
         double desiredToEnd = desiredOrientation.distance(endOrientation, true);
         double tolerance = action.getState().getOrientationDistanceToGoalTolerance();
         double error = Math.abs(currentToEnd - desiredToEnd);
         int dataColor = error < tolerance ? ImGuiTools.GREEN : ImGuiTools.RED;

         if (action.getState().getIsExecuting())
         {
            currentOrientationPlotLine.setDataColor(dataColor);
            currentOrientationPlotLine.addValue(Math.toDegrees(currentToEnd));
            desiredOrientationPlotLine.setDataColor(ImGuiTools.GRAY);
            desiredOrientationPlotLine.addValue(Math.toDegrees(desiredToEnd));
         }
         if (renderAsPlots)
         {
            orientationErrorPlot.render(dividedBarWidth, PLOT_HEIGHT);
         }
         else
         {
            double barEndValue = Math.max(Math.min(initialToEnd, currentToEnd), 2.0 * tolerance);
            double toleranceMarkPercent = tolerance / barEndValue;
            double percentLeft = currentToEnd / barEndValue;
            ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT,
                                         dividedBarWidth,
                                         dataColor,
                                         percentLeft,
                                         toleranceMarkPercent,
                                         "%.2f / %.2f".formatted(Math.toDegrees(currentToEnd), Math.toDegrees(initialToEnd)));
         }
      }
      else
      {
         renderBlankProgress("Empty Orientation Error", dividedBarWidth, renderAsPlots, true);
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
      FootstepPlanActionStateBasics footstepPlanActionState = null;
      if (action instanceof RDXWalkAction walkAction)
         footstepPlanActionState = walkAction.getState().getBasics();
      if (action instanceof RDXFootstepPlanAction footstepPlanAction)
         footstepPlanActionState = footstepPlanAction.getState().getBasics();

      if (footstepPlanActionState != null && footstepPlanActionState.getTotalNumberOfFootsteps() > 0)
      {
         double percentLeft = footstepPlanActionState.getNumberOfIncompleteFootsteps() / (double) footstepPlanActionState.getTotalNumberOfFootsteps();
         String overlay = "%d / %d".formatted(footstepPlanActionState.getNumberOfIncompleteFootsteps(), footstepPlanActionState.getTotalNumberOfFootsteps());

         if (action.getState().getIsExecuting())
         {
            footstepsRemainingPlotLine.addValue(footstepPlanActionState.getNumberOfIncompleteFootsteps());
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
