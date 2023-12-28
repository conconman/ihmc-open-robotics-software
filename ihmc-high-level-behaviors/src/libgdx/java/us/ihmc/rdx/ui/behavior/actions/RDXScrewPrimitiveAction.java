package us.ihmc.rdx.ui.behavior.actions;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionDefinition;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.mesh.RDXDashedLineMesh;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXTrajectoryGraphic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXScrewPrimitiveAction extends RDXActionNode<ScrewPrimitiveActionState, ScrewPrimitiveActionDefinition>
{
   private final FullHumanoidRobotModel syncedRobot;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiReferenceFrameLibraryCombo objectFrameComboBox;
   private final ImDoubleWrapper translationWidget;
   private final ImDoubleWrapper rotationWidget;
   private final ImDoubleWrapper maxLinearVelocityWidget;
   private final ImDoubleWrapper maxAngularVelocityWidget;
   private final ImDoubleWrapper maxForceWidget;
   private final ImDoubleWrapper maxTorqueWidget;
   private final ImDoubleWrapper linearPositionWeightWidget;
   private final ImDoubleWrapper angularPositionWeightWidget;
   private final RDXSelectablePose3DGizmo screwAxisGizmo;
   private final ImBoolean adjustWrenchContactPose = new ImBoolean();
   private final RDXSelectablePose3DGizmo wrenchContactPoseGizmo;
   private final RDXDashedLineMesh screwAxisGraphic = new RDXDashedLineMesh(Color.WHITE, Axis3D.X, 0.04);
   private final RDXTrajectoryGraphic trajectoryGraphic = new RDXTrajectoryGraphic();
   private final RecyclingArrayList<FramePose3D> trajectoryPoses = new RecyclingArrayList<>(FramePose3D::new);

   public RDXScrewPrimitiveAction(long id,
                                  CRDTInfo crdtInfo,
                                  WorkspaceResourceDirectory saveFileDirectory,
                                  RDX3DPanel panel3D,
                                  DRCRobotModel robotModel,
                                  FullHumanoidRobotModel syncedRobot,
                                  RobotCollisionModel selectionCollisionModel,
                                  ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(new ScrewPrimitiveActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      this.syncedRobot = syncedRobot;

      getDefinition().setDescription("Screw primitive");

      screwAxisGizmo = new RDXSelectablePose3DGizmo(getDefinition().getScrewAxisPoseInObjectFrame().getValue(),
                                                    ReferenceFrame.getWorldFrame(),
                                                    getSelected());
      screwAxisGizmo.create(panel3D);
      wrenchContactPoseGizmo = new RDXSelectablePose3DGizmo(getDefinition().getWrenchContactPoseInHandControlFrame().getValue(),
                                                            ReferenceFrame.getWorldFrame(),
                                                            adjustWrenchContactPose);
      wrenchContactPoseGizmo.create(panel3D);

      objectFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Object frame",
                                                                referenceFrameLibrary,
                                                                getDefinition()::getObjectFrameName,
                                                                getDefinition()::setObjectFrameName);
      translationWidget = new ImDoubleWrapper(getDefinition()::getTranslation,
                                              getDefinition()::setTranslation,
                                              imDouble -> ImGuiTools.sliderDouble(labels.get("Translation"), imDouble, -0.4, 0.4));
      rotationWidget = new ImDoubleWrapper(getDefinition()::getRotation,
                                           getDefinition()::setRotation,
                                           imDouble -> ImGuiTools.sliderDouble(labels.get("Rotation"), imDouble, -2.0 * Math.PI, 2.0 * Math.PI));
      maxLinearVelocityWidget = new ImDoubleWrapper(getDefinition()::getMaxLinearVelocity,
                                                    getDefinition()::setMaxLinearVelocity,
                                                    imDouble -> ImGuiTools.sliderDouble(labels.get("Max Linear Velocity"), imDouble, 0.05, 1.0));
      maxAngularVelocityWidget = new ImDoubleWrapper(getDefinition()::getMaxAngularVelocity,
                                                     getDefinition()::setMaxAngularVelocity,
                                                     imDouble -> ImGuiTools.sliderDouble(labels.get("Max Angular Velocity"), imDouble, 0.1, Math.PI));
      maxForceWidget = new ImDoubleWrapper(getDefinition()::getMaxForce,
                                           getDefinition()::setMaxForce,
                                           imDouble -> ImGuiTools.sliderDouble(labels.get("Max Force"), imDouble, 1.0, 70.0));
      maxTorqueWidget = new ImDoubleWrapper(getDefinition()::getMaxTorque,
                                            getDefinition()::setMaxTorque,
                                            imDouble -> ImGuiTools.sliderDouble(labels.get("Max Torque"), imDouble, 0.5, 20.0));
      linearPositionWeightWidget = new ImDoubleWrapper(getDefinition()::getLinearPositionWeight,
                                                       getDefinition()::setLinearPositionWeight,
                                                       imDouble -> ImGuiTools.sliderDouble(labels.get("Linear Position Weight"), imDouble, 0.0, 100.0));
      angularPositionWeightWidget = new ImDoubleWrapper(getDefinition()::getAngularPositionWeight,
                                                        getDefinition()::setAngularPositionWeight,
                                                        imDouble -> ImGuiTools.sliderDouble(labels.get("Angular Position Weight"), imDouble, 0.0, 100.0));
   }

   @Override
   public void update()
   {
      super.update();

      if (getState().getScrewFrame().isChildOfWorld())
      {
         screwAxisGizmo.getPoseGizmo().setGizmoFrame(getState().getScrewFrame().getReferenceFrame());
         screwAxisGizmo.getPoseGizmo().update();
         ReferenceFrame handControlFrame = syncedRobot.getHandControlFrame(getDefinition().getSide());
         if (wrenchContactPoseGizmo.getPoseGizmo().getGizmoFrame().getParent() != handControlFrame)
         {
            wrenchContactPoseGizmo.getPoseGizmo().setParentFrame(handControlFrame);
         }
         wrenchContactPoseGizmo.getPoseGizmo().update();

         double screwAxisLineWidth = 0.005;
         screwAxisGraphic.update(screwAxisGizmo.getPoseGizmo().getPose(), screwAxisLineWidth, 1.0);

         double trajectoryLineWidth = 0.01;
         trajectoryPoses.clear();
         for (int i = 0; i < getState().getTrajectory().getSize(); i++)
            trajectoryPoses.add().set(getState().getTrajectory().getValueReadOnly(i));
         trajectoryGraphic.update(trajectoryLineWidth, trajectoryPoses);
      }
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      objectFrameComboBox.render();
      translationWidget.renderImGuiWidget();
      rotationWidget.renderImGuiWidget();
      maxLinearVelocityWidget.renderImGuiWidget();
      maxAngularVelocityWidget.renderImGuiWidget();
      maxForceWidget.renderImGuiWidget();
      maxTorqueWidget.renderImGuiWidget();
      linearPositionWeightWidget.renderImGuiWidget();
      angularPositionWeightWidget.renderImGuiWidget();
      ImGui.checkbox(labels.get("Adjust Wrench Contact Pose"), adjustWrenchContactPose);
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (getState().getScrewFrame().isChildOfWorld())
      {
         screwAxisGizmo.calculate3DViewPick(input);
         wrenchContactPoseGizmo.calculate3DViewPick(input);
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (getState().getScrewFrame().isChildOfWorld())
      {
         screwAxisGizmo.process3DViewInput(input);
         wrenchContactPoseGizmo.process3DViewInput(input);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (getState().getScrewFrame().isChildOfWorld())
      {
         screwAxisGizmo.getVirtualRenderables(renderables, pool);
         wrenchContactPoseGizmo.getVirtualRenderables(renderables, pool);
         screwAxisGraphic.getRenderables(renderables, pool);
         trajectoryGraphic.getRenderables(renderables, pool);
      }
   }

   @Override
   public String getActionTypeTitle()
   {
      return getDefinition().getSide().getPascalCaseName() + " Screw Primitive";
   }
}
