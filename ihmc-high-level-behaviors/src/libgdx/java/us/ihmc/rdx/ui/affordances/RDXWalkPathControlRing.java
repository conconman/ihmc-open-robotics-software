package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.BehaviorTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.footstepPlanner.RDXFootstepPlanning;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePathControlRingGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class RDXWalkPathControlRing
{
   private final RDXSelectablePathControlRingGizmo footstepPlannerGoalGizmo = new RDXSelectablePathControlRingGizmo();
   private final Notification becomesModifiedNotification = new Notification();
   private RDX3DPanel panel3D;
   private MovingReferenceFrame midFeetZUpFrame;
   private RDXFootstepGraphic leftStanceFootstepGraphic;
   private RDXFootstepGraphic rightStanceFootstepGraphic;
   private RDXFootstepGraphic leftGoalFootstepGraphic;
   private RDXFootstepGraphic rightGoalFootstepGraphic;
   private final FramePose3D leftStanceFootPose = new FramePose3D();
   private final FramePose3D rightStanceFootPose = new FramePose3D();
   private final FramePose3D leftGoalFootPose = new FramePose3D();
   private final FramePose3D rightGoalFootPose = new FramePose3D();
   private ReferenceFrame goalFrame;
   private final FramePose3D goalPose = new FramePose3D();
   private final FramePose3D midFeetZUpPose = new FramePose3D();
   private SideDependentList<MovingReferenceFrame> footFrames;
   private double halfIdealFootstepWidth;
   private final AxisAngle walkFacingDirection = new AxisAngle();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImGui3DViewInput latestInput;
   private RDXFootstepPlanning footstepPlanning;

   public void create(RDX3DPanel panel3D,
                      DRCRobotModel robotModel,
                      ROS2SyncedRobotModel syncedRobot,
                      FootstepPlannerParametersBasics footstepPlannerParameters,
                      RDXFootstepPlanning footstepPlanning)
   {
      this.panel3D = panel3D;
      this.footstepPlanning = footstepPlanning;

      footstepPlannerGoalGizmo.create(panel3D);
      panel3D.addImGuiOverlayAddition(this::renderTooltips);
      midFeetZUpFrame = syncedRobot.getReferenceFrames().getMidFeetZUpFrame();
      footFrames = syncedRobot.getReferenceFrames().getSoleFrames();

      SegmentDependentList<RobotSide, ArrayList<Point2D>> contactPoints = robotModel.getContactPointParameters().getControllerFootGroundContactPoints();
      leftStanceFootstepGraphic = new RDXFootstepGraphic(contactPoints, RobotSide.LEFT);
      rightStanceFootstepGraphic = new RDXFootstepGraphic(contactPoints, RobotSide.RIGHT);
      leftGoalFootstepGraphic = new RDXFootstepGraphic(contactPoints, RobotSide.LEFT);
      rightGoalFootstepGraphic = new RDXFootstepGraphic(contactPoints, RobotSide.RIGHT);

      goalFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("goalPose",
                                                                                  ReferenceFrame.getWorldFrame(),
                                                                                  footstepPlannerGoalGizmo.getPathControlRingGizmo().getTransformToParent());

      leftStanceFootstepGraphic.create();
      rightStanceFootstepGraphic.create();
      leftGoalFootstepGraphic.create();
      rightGoalFootstepGraphic.create();

      halfIdealFootstepWidth = footstepPlannerParameters.getIdealFootstepWidth() / 2.0;
      leftStanceFootPose.getPosition().addY(halfIdealFootstepWidth);
      rightStanceFootPose.getPosition().subY(halfIdealFootstepWidth);
      leftStanceFootstepGraphic.setPose(leftStanceFootPose);
      rightStanceFootstepGraphic.setPose(rightStanceFootPose);
   }

   public void update()
   {
      if (!footstepPlannerGoalGizmo.getModified())
      {
         footstepPlannerGoalGizmo.getPathControlRingGizmo().getTransformToParent().set(midFeetZUpFrame.getTransformToWorldFrame());
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      footstepPlannerGoalGizmo.calculate3DViewPick(input);
   }

   // This happens after update.
   public void process3DViewInput(ImGui3DViewInput input)
   {
      latestInput = input;
      boolean leftMouseReleasedWithoutDrag = input.isWindowHovered() && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);

      boolean previouslySelected = footstepPlannerGoalGizmo.getSelected();
      footstepPlannerGoalGizmo.process3DViewInput(input);
      boolean newlySelected = footstepPlannerGoalGizmo.getSelected() && !previouslySelected;

      if (newlySelected)
      {
         becomeModified(true);
      }

      if (footstepPlannerGoalGizmo.getModified())
      {
         updateStuff();
      }
      if (footstepPlannerGoalGizmo.getSelected() && leftMouseReleasedWithoutDrag)
      {
         if (footstepPlannerGoalGizmo.getPathControlRingGizmo().getPositiveXArrowHovered())
         {
            walkFacingDirection.set(Axis3D.Z, 0.0);
         }
         else if (footstepPlannerGoalGizmo.getPathControlRingGizmo().getPositiveYArrowHovered())
         {
            walkFacingDirection.set(Axis3D.Z, Math.PI / 2.0);
         }
         else if (footstepPlannerGoalGizmo.getPathControlRingGizmo().getNegativeXArrowHovered())
         {
            walkFacingDirection.set(Axis3D.Z, Math.PI);
         }
         else if (footstepPlannerGoalGizmo.getPathControlRingGizmo().getNegativeYArrowHovered())
         {
            walkFacingDirection.set(Axis3D.Z, -Math.PI / 2.0);
         }
         if (footstepPlannerGoalGizmo.getPathControlRingGizmo().getAnyArrowHovered())
         {
            footstepPlannerGoalGizmo.getPathControlRingGizmo().getTransformToParent().appendOrientation(walkFacingDirection);
            updateStuff();
            queueFootstepPlan();
         }
      }
      if (footstepPlannerGoalGizmo.getSelected() && footstepPlannerGoalGizmo.getPathControlRingGizmo().getGizmoModifiedByUser().poll())
      {
         queueFootstepPlan();
      }

      if (footstepPlannerGoalGizmo.getModified() && footstepPlannerGoalGizmo.getSelected() && ImGui.isKeyReleased(ImGuiTools.getDeleteKey()))
      {
         delete();
      }
      if (footstepPlannerGoalGizmo.getSelected() && ImGui.isKeyReleased(ImGuiTools.getEscapeKey()))
      {
         footstepPlannerGoalGizmo.setSelected(false);
      }
   }

   private void queueFootstepPlan()
   {
      footstepPlanning.queueAsynchronousPlanning(goalPose);
   }

   private void updateStuff()
   {
      leftStanceFootPose.setToZero(footFrames.get(RobotSide.LEFT));
      leftStanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightStanceFootPose.setToZero(footFrames.get(RobotSide.RIGHT));
      rightStanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      double lowestStanceZ = Math.min(leftStanceFootPose.getZ(), rightStanceFootPose.getZ());
      leftStanceFootPose.setZ(lowestStanceZ);
      rightStanceFootPose.setZ(lowestStanceZ);

      goalFrame.update();
      goalPose.setToZero(goalFrame);

      leftGoalFootPose.setIncludingFrame(goalPose);
      leftGoalFootPose.getPosition().addY(halfIdealFootstepWidth);
      leftGoalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      leftGoalFootPose.setZ(lowestStanceZ);
      rightGoalFootPose.setIncludingFrame(goalPose);
      rightGoalFootPose.getPosition().subY(halfIdealFootstepWidth);
      rightGoalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightGoalFootPose.setZ(lowestStanceZ);

      goalPose.changeFrame(ReferenceFrame.getWorldFrame());
      goalPose.setZ(lowestStanceZ);

      midFeetZUpPose.setToZero(midFeetZUpFrame);
      midFeetZUpPose.changeFrame(ReferenceFrame.getWorldFrame());

      leftGoalFootstepGraphic.setPose(leftGoalFootPose);
      rightGoalFootstepGraphic.setPose(rightGoalFootPose);
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Control ring:");
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Deleted"), !footstepPlannerGoalGizmo.getSelected() && !footstepPlannerGoalGizmo.getModified()))
      {
         delete();
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Modified"), !footstepPlannerGoalGizmo.getSelected() && footstepPlannerGoalGizmo.getModified()))
      {
         becomeModified(false);
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Selected"), footstepPlannerGoalGizmo.getSelected() && footstepPlannerGoalGizmo.getModified()))
      {
         becomeModified(true);
      }
   }

   public void becomeModified(boolean selected)
   {
      footstepPlannerGoalGizmo.setSelected(selected);
      if (!footstepPlannerGoalGizmo.getModified())
      {
         footstepPlannerGoalGizmo.setModified(true);
         becomesModifiedNotification.set();
         walkFacingDirection.set(Axis3D.Z, 0.0);
         updateStuff();
         queueFootstepPlan();
      }
   }

   private void renderTooltips()
   {
      if (footstepPlannerGoalGizmo.getSelected() && footstepPlannerGoalGizmo.getPathControlRingGizmo().getAnyPartHovered())
      {
         float offsetX = 10.0f;
         float offsetY = 10.0f;
         float mousePosX = latestInput.getMousePosX();
         float mousePosY = latestInput.getMousePosY();
         float drawStartX = panel3D.getWindowDrawMinX() + mousePosX + offsetX;
         float drawStartY = panel3D.getWindowDrawMinY() + mousePosY + offsetY;

         String message = """
                          Use left mouse drag to translate.
                          Use right mouse drag to yaw.
                          Use keyboard arrows to translate. (Hold shift for slow)
                          Use alt+left and alt+left arrows to yaw. (Hold shift for slow)
                          """;

         ImGui.getWindowDrawList()
              .addRectFilled(drawStartX, drawStartY, drawStartX + 62 * 6.7f, drawStartY + 4 * 17.0f, new Color(0.2f, 0.2f, 0.2f, 0.7f).toIntBits());
         ImGui.getWindowDrawList()
              .addText(ImGuiTools.getSmallFont(),
                       ImGuiTools.getSmallFont().getFontSize(),
                       drawStartX + 5.0f,
                       drawStartY + 2.0f,
                       Color.WHITE.toIntBits(),
                       message);
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (footstepPlannerGoalGizmo.getModified())
      {
         leftStanceFootstepGraphic.getRenderables(renderables, pool);
         rightStanceFootstepGraphic.getRenderables(renderables, pool);
         leftGoalFootstepGraphic.getRenderables(renderables, pool);
         rightGoalFootstepGraphic.getRenderables(renderables, pool);
      }
      footstepPlannerGoalGizmo.getVirtualRenderables(renderables, pool);
   }

   public void delete()
   {
      footstepPlannerGoalGizmo.setSelected(false);
      footstepPlannerGoalGizmo.setModified(false);
      clearGraphics();
   }

   public void clearGraphics()
   {
      leftStanceFootstepGraphic.setPose(BehaviorTools.createNaNPose());
      rightStanceFootstepGraphic.setPose(BehaviorTools.createNaNPose());
      leftGoalFootstepGraphic.setPose(BehaviorTools.createNaNPose());
      rightGoalFootstepGraphic.setPose(BehaviorTools.createNaNPose());
   }

   public boolean isSelected()
   {
      return footstepPlannerGoalGizmo.getSelected();
   }

   public Notification getBecomesModifiedNotification()
   {
      return becomesModifiedNotification;
   }
}
