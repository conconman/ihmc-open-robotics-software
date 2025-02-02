package us.ihmc.rdx.ui.behavior.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.actions.WalkActionDefinition;
import us.ihmc.behaviors.sequence.actions.WalkActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePathControlRingGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXWalkAction extends RDXActionNode<WalkActionState, WalkActionDefinition>
{
   private final WalkActionState state;
   private final RDXFootstepPlanGraphic footstepPlanGraphic;
   private final ImGuiReferenceFrameLibraryCombo parentFrameComboBox;
   private final SideDependentList<RDXFootstepGraphic> goalFeetGraphics = new SideDependentList<>();
   private final RDXSelectablePathControlRingGizmo footstepPlannerGoalGizmo;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final SideDependentList<ImBoolean> goalFeetPosesSelected = new SideDependentList<>();
   private final SideDependentList<RDXPose3DGizmo> goalFeetGizmos = new SideDependentList<>();
   private final ImBooleanWrapper executeWithNextActionWrapper;
   private final ImDoubleWrapper swingDurationWidget;
   private final ImDoubleWrapper transferDurationWidget;
   private final RDX3DPanelTooltip tooltip;

   public RDXWalkAction(long id,
                        CRDTInfo crdtInfo,
                        WorkspaceResourceDirectory saveFileDirectory,
                        RDX3DPanel panel3D,
                        DRCRobotModel robotModel,
                        ReferenceFrameLibrary referenceFrameLibrary,
                        FootstepPlannerParametersBasics footstepPlannerParameters)
   {
      super(new WalkActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();

      getDefinition().setDescription("Walk");

      for (RobotSide side : RobotSide.values)
      {
         getDefinition().getGoalFootstepToGoalTransform(side).getValue()
                        .getTranslation().addY(0.5 * side.negateIfRightSide(footstepPlannerParameters.getIdealFootstepWidth()));
      }

      footstepPlannerGoalGizmo = new RDXSelectablePathControlRingGizmo(ReferenceFrame.getWorldFrame(),
                                                                       getDefinition().getGoalToParentTransform().getValue(),
                                                                       getSelected());
      footstepPlannerGoalGizmo.create(panel3D);
      footstepPlanGraphic = new RDXFootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());

      parentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Parent frame",
                                                                referenceFrameLibrary,
                                                                getDefinition().getBasics()::getParentFrameName,
                                                                getState().getGoalFrame()::changeFrame);
      executeWithNextActionWrapper = new ImBooleanWrapper(getDefinition()::getExecuteWithNextAction,
                                                          getDefinition()::setExecuteWithNextAction,
                                                          imBoolean -> ImGui.checkbox(labels.get("Execute with next action"), imBoolean));
      swingDurationWidget = new ImDoubleWrapper(getDefinition().getBasics()::getSwingDuration,
                                                getDefinition().getBasics()::setSwingDuration,
                                                imDouble -> ImGui.inputDouble(labels.get("Swing duration"), imDouble));
      transferDurationWidget = new ImDoubleWrapper(getDefinition().getBasics()::getTransferDuration,
                                                   getDefinition().getBasics()::setTransferDuration,
                                                   imDouble -> ImGui.inputDouble(labels.get("Transfer duration"), imDouble));

      for (RobotSide side : RobotSide.values)
      {
         goalFeetPosesSelected.put(side, new ImBoolean(false));

         RDXPose3DGizmo footGizmo = new RDXPose3DGizmo(ReferenceFrame.getWorldFrame(),
                                                       getDefinition().getGoalFootstepToGoalTransform(side).getValue());
         footGizmo.create(panel3D);
         goalFeetGizmos.put(side, footGizmo);

         RDXFootstepGraphic goalFootGraphic = new RDXFootstepGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints(), side);
         goalFootGraphic.create();
         goalFeetGraphics.put(side, goalFootGraphic);
      }

      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);
   }

   @Override
   public void update()
   {
      super.update();

      if (state.getGoalFrame().isChildOfWorld())
      {
         if (footstepPlannerGoalGizmo.getPathControlRingGizmo().getGizmoFrame() != state.getGoalFrame().getReferenceFrame())
         {
            footstepPlannerGoalGizmo.getPathControlRingGizmo().setGizmoFrame(state.getGoalFrame().getReferenceFrame());
         }

         for (RobotSide side : RobotSide.values)
         {
            if (goalFeetGizmos.get(side).getGizmoFrame().getParent() != state.getGoalFrame().getReferenceFrame())
            {
               goalFeetGizmos.get(side).setParentFrame(state.getGoalFrame().getReferenceFrame());
            }
         }

         if (!getSelected().get())
            goalFeetPosesSelected.forEach(imBoolean -> imBoolean.set(false));

         footstepPlannerGoalGizmo.getPathControlRingGizmo().update();
         for (RobotSide side : RobotSide.values)
         {
            goalFeetGizmos.get(side).update();
            goalFeetGraphics.get(side).setPose(goalFeetGizmos.get(side).getPose());
         }
         footstepPlanGraphic.update();
      }
   }

   @Override
   public void calculateVRPick(RDXVRContext vrContext)
   {
      if (state.getGoalFrame().isChildOfWorld())
      {
         footstepPlannerGoalGizmo.calculateVRPick(vrContext);
      }
   }

   @Override
   public void processVRInput(RDXVRContext vrContext)
   {
      if (state.getGoalFrame().isChildOfWorld())
      {
         footstepPlannerGoalGizmo.processVRInput(vrContext);
      }
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (state.getGoalFrame().isChildOfWorld())
      {
         footstepPlannerGoalGizmo.calculate3DViewPick(input);
         if (getSelected().get())
         {
            for (RobotSide side : RobotSide.values)
            {
               if (goalFeetPosesSelected.get(side).get())
               {
                  goalFeetGizmos.get(side).calculate3DViewPick(input);
               }
            }
         }
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (state.getGoalFrame().isChildOfWorld())
      {
         footstepPlannerGoalGizmo.process3DViewInput(input);
         tooltip.setInput(input);
         if (getSelected().get())
         {
            for (RobotSide side : RobotSide.values)
            {
               if (goalFeetPosesSelected.get(side).get())
               {
                  goalFeetGizmos.get(side).process3DViewInput(input);
               }
            }
         }
      }
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.sameLine();
      executeWithNextActionWrapper.renderImGuiWidget();
      parentFrameComboBox.render();
      if (ImGui.button(labels.get("Plan")))
      {
         // TODO: Plan preview message
      }
      if (state.getGoalFrame().isChildOfWorld()) // Prevent editing footsteps unless we are a child of world
      {
         ImGui.sameLine();
         for (RobotSide side : RobotSide.values)
         {
            ImGui.checkbox(labels.get("Edit " + side.getPascalCaseName()), goalFeetPosesSelected.get(side));
            if (side == RobotSide.LEFT)
               ImGui.sameLine();
         }
      }
      ImGui.pushItemWidth(80.0f);
      swingDurationWidget.renderImGuiWidget();
      transferDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   public void render3DPanelImGuiOverlays()
   {
      if (footstepPlannerGoalGizmo.getPathControlRingGizmo().getRingHovered())
      {
         tooltip.render("%s Action\nIndex: %d\nDescription: %s".formatted(getActionTypeTitle(),
                                                                          state.getActionIndex(),
                                                                          getDefinition().getDescription()));
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (state.getGoalFrame().isChildOfWorld())
      {
         footstepPlanGraphic.getRenderables(renderables, pool);
         footstepPlannerGoalGizmo.getVirtualRenderables(renderables, pool);
         if (getSelected().get())
         {
            for (RobotSide side : RobotSide.values)
            {
               if (goalFeetPosesSelected.get(side).get())
               {
                  goalFeetGizmos.get(side).getRenderables(renderables, pool);
               }
            }
         }
         for (RobotSide side : RobotSide.values)
            goalFeetGraphics.get(side).getRenderables(renderables, pool);
      }
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Walk Goal";
   }
}
