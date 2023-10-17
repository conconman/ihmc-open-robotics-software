package us.ihmc.rdx.ui.behavior.actions;

import behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.behaviors.sequence.IKRootCalculator;
import us.ihmc.behaviors.sequence.actions.HandPoseActionDefinition;
import us.ihmc.behaviors.sequence.actions.HandPoseActionState;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.simulation.scs2.RDXFrameNodePart;
import us.ihmc.rdx.simulation.scs2.RDXMultiBodySystemFactories;
import us.ihmc.rdx.simulation.scs2.RDXRigidBody;
import us.ihmc.rdx.simulation.scs2.RDXVisualTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.affordances.RDXInteractableHighlightModel;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.behavior.sequence.RDXBehaviorAction;
import us.ihmc.rdx.ui.behavior.sequence.RDXBehaviorActionSequenceEditor;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.interaction.MouseCollidable;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.wholeBodyController.HandTransformTools;

import java.util.ArrayList;
import java.util.List;

public class RDXHandPoseAction extends RDXBehaviorAction
{
   public static final String GOOD_QUALITY_COLOR = "0x4B61D1";
   public static final String BAD_QUALITY_COLOR = "0xD14B4B";
   private final ROS2SyncedRobotModel syncedRobot;
   private final HandPoseActionState state;
   private final HandPoseActionDefinition definition;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   /** Gizmo is control frame */
   private final RDXSelectablePose3DGizmo poseGizmo;
   private final SideDependentList<String> handNames = new SideDependentList<>();
   private final MutableReferenceFrame graphicFrame = new MutableReferenceFrame();
   private final MutableReferenceFrame collisionShapeFrame = new MutableReferenceFrame();
   private final Color goodQualityColor;
   private final Color badQualityColor;
   private boolean isMouseHovering = false;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final ArrayList<MouseCollidable> mouseCollidables = new ArrayList<>();
   private final SideDependentList<RDXInteractableHighlightModel> highlightModels = new SideDependentList<>();
   private final ImGuiReferenceFrameLibraryCombo parentFrameComboBox;
   private final ImDoubleWrapper trajectoryDurationWidget;
   private final ImBooleanWrapper executeWithNextActionWrapper;
   private final ImBooleanWrapper holdPoseInWorldLaterWrapper;
   private final ImBooleanWrapper jointSpaceControlWrapper;
   private final SideDependentList<RDXRigidBody> armMultiBodyGraphics = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics[]> armGraphicOneDoFJoints = new SideDependentList<>();
   private final SideDependentList<Color> currentColor = new SideDependentList<>();
   private final IHMCROS2Input<HandPoseJointAnglesStatusMessage> leftHandJointAnglesStatusSubscription;
   private final IHMCROS2Input<HandPoseJointAnglesStatusMessage> rightHandJointAnglesStatusSubscription;
   private final RDX3DPanelTooltip tooltip;
   private final IKRootCalculator rootCalculator;

   public RDXHandPoseAction(RDXBehaviorActionSequenceEditor editor,
                            RDX3DPanel panel3D,
                            DRCRobotModel robotModel,
                            ROS2SyncedRobotModel syncedRobot,
                            RobotCollisionModel selectionCollisionModel,
                            ROS2ControllerPublishSubscribeAPI ros2)
   {
      super(editor);

      this.syncedRobot = syncedRobot;

      state = new HandPoseActionState(syncedRobot);
      definition = state.getDefinition();

      poseGizmo = new RDXSelectablePose3DGizmo(ReferenceFrame.getWorldFrame(), definition.getPalmTransformToParent(), getSelected());
      poseGizmo.create(panel3D);

      trajectoryDurationWidget = new ImDoubleWrapper(definition::getTrajectoryDuration,
                                                     definition::setTrajectoryDuration,
                                                     imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));
      executeWithNextActionWrapper = new ImBooleanWrapper(definition::getExecuteWithNextAction,
                                                          definition::setExecuteWithNextAction,
                                                          imBoolean -> ImGui.checkbox(labels.get("Execute with next action"), imBoolean));
      holdPoseInWorldLaterWrapper = new ImBooleanWrapper(definition::getHoldPoseInWorldLater,
                                                         definition::setHoldPoseInWorldLater,
                                                         imBoolean -> ImGui.checkbox(labels.get("Hold pose in world later"), imBoolean));
      jointSpaceControlWrapper = new ImBooleanWrapper(definition::getJointSpaceControl,
                                                      definition::setJointSpaceControl,
                                                      imBoolean -> {
                                                         if (ImGui.radioButton(labels.get("Joint space"), imBoolean.get()))
                                                            imBoolean.set(true);
                                                         ImGui.sameLine();
                                                         if (ImGui.radioButton(labels.get("Task space"), !imBoolean.get()))
                                                            imBoolean.set(false);
                                                      });

      ColorDefinition goodQualityColorDefinition = ColorDefinitions.parse(GOOD_QUALITY_COLOR).derive(0.0, 1.0, 1.0, 0.5);
      goodQualityColor = RDXVisualTools.toColor(goodQualityColorDefinition);
      badQualityColor = RDXVisualTools.toColor(ColorDefinitions.parse(BAD_QUALITY_COLOR).derive(0.0, 1.0, 1.0, 0.5));

      for (RobotSide side : RobotSide.values)
      {
         handNames.put(side, syncedRobot.getFullRobotModel().getHand(side).getName());

         RigidBodyTransformReadOnly graphicToControlFrameTransform = HandTransformTools.getHandGraphicToControlFrameTransform(syncedRobot.getFullRobotModel(),
                                                                                                                              robotModel.getUIParameters(),
                                                                                                                              side);
         graphicFrame.update(transformToParent -> transformToParent.set(graphicToControlFrameTransform));

         String handBodyName = handNames.get(side);
         String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(handBodyName));
         highlightModels.put(side, new RDXInteractableHighlightModel(modelFileName));

         MultiBodySystemBasics handOnlySystem = MultiBodySystemMissingTools.createSingleBodySystem(syncedRobot.getFullRobotModel().getHand(side));
         List<Collidable> handCollidables = selectionCollisionModel.getRobotCollidables(handOnlySystem);

         RigidBodyTransformReadOnly linkToControlFrameTransform = HandTransformTools.getHandLinkToControlFrameTransform(syncedRobot.getFullRobotModel(), side);
         collisionShapeFrame.update(transformToParent -> transformToParent.set(linkToControlFrameTransform));

         for (Collidable handCollidable : handCollidables)
         {
            mouseCollidables.add(new MouseCollidable(handCollidable));
         }

         HumanoidJointNameMap jointMap = robotModel.getJointMap();
         ArmJointName firstArmJointName = jointMap.getArmJointNames()[0];
         RobotDefinition armDefinition = RobotDefinitionTools.cloneLimbOnlyDefinitionWithElevator(robotModel.getRobotDefinition(),
                                                                                                  jointMap.getChestName(),
                                                                                                  jointMap.getArmJointName(side, firstArmJointName));
         MaterialDefinition material = new MaterialDefinition(goodQualityColorDefinition);
         currentColor.put(side, goodQualityColor);
         RobotDefinition.forEachRigidBodyDefinition(armDefinition.getRootBodyDefinition(), body ->
         {
            body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material));
         });
         RigidBodyBasics armOnlyMultiBody
               = MultiBodySystemMissingTools.getDetachedCopyOfSubtreeWithElevator(syncedRobot.getFullRobotModel().getChest(),
                                                                                  syncedRobot.getFullRobotModel().getArmJoint(side, firstArmJointName));
         armMultiBodyGraphics.put(side,
                                  RDXMultiBodySystemFactories.toRDXMultiBodySystem(armOnlyMultiBody, armDefinition, RDXVisualTools.DESIRED_ROBOT_SCALING));
         armMultiBodyGraphics.get(side).getRigidBodiesToHide().add("elevator");
         armMultiBodyGraphics.get(side).getRigidBodiesToHide().add(jointMap.getChestName());
         armGraphicOneDoFJoints.put(side, MultiBodySystemMissingTools.getSubtreeJointArray(OneDoFJointBasics.class, armMultiBodyGraphics.get(side)));
      }

      parentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Parent frame",
                                                                definition::getPalmParentFrameName,
                                                                definition::setPalmParentFrameName);

      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);

      leftHandJointAnglesStatusSubscription = ros2.subscribe(BehaviorActionSequence.LEFT_HAND_POSE_JOINT_ANGLES_STATUS);
      rightHandJointAnglesStatusSubscription = ros2.subscribe(BehaviorActionSequence.RIGHT_HAND_POSE_JOINT_ANGLES_STATUS);
      rootCalculator = new IKRootCalculator(ros2, syncedRobot);
   }

   @Override
   public void update()
   {
      super.update();

      if (state.getPalmFrame().isChildOfWorld())
      {
         if (poseGizmo.getPoseGizmo().getGizmoFrame() != state.getPalmFrame().getReferenceFrame())
         {
            poseGizmo.getPoseGizmo().setGizmoFrame(state.getPalmFrame().getReferenceFrame());
            graphicFrame.setParentFrame(state.getPalmFrame().getReferenceFrame());
            collisionShapeFrame.setParentFrame(state.getPalmFrame().getReferenceFrame());
         }

         poseGizmo.getPoseGizmo().update();
         highlightModels.get(definition.getSide()).setPose(graphicFrame.getReferenceFrame());

         if (poseGizmo.isSelected() || isMouseHovering)
         {
            highlightModels.get(definition.getSide()).setTransparency(0.7);
         }
         else
         {
            highlightModels.get(definition.getSide()).setTransparency(0.5);
         }

         // IK solution visualization via ghost arms
         if (state.getIsNextForExecution())
            visualizeIK();
      }
   }

   private void visualizeIK()
   {
      boolean receivedDataForThisSide = (definition.getSide() == RobotSide.LEFT && leftHandJointAnglesStatusSubscription.hasReceivedFirstMessage()) ||
                                        (definition.getSide() == RobotSide.RIGHT && rightHandJointAnglesStatusSubscription.hasReceivedFirstMessage());
      if (receivedDataForThisSide)
      {
         HandPoseJointAnglesStatusMessage handPoseJointAnglesStatusMessage;
         if (definition.getSide() == RobotSide.LEFT)
            handPoseJointAnglesStatusMessage = leftHandJointAnglesStatusSubscription.getLatest();
         else
            handPoseJointAnglesStatusMessage = rightHandJointAnglesStatusSubscription.getLatest();
         if (handPoseJointAnglesStatusMessage.getActionInformation().getActionIndex() == state.getActionIndex())
         {
            SixDoFJoint floatingJoint = (SixDoFJoint) armMultiBodyGraphics.get(definition.getSide()).getRigidBody().getChildrenJoints().get(0);
            rootCalculator.getKinematicsInfo();
            rootCalculator.computeRoot();
            floatingJoint.getJointPose().set(rootCalculator.getRoot().getTransformToRoot());

            for (int i = 0; i < handPoseJointAnglesStatusMessage.getJointAngles().length; i++)
            {
               armGraphicOneDoFJoints.get(definition.getSide())[i].setQ(handPoseJointAnglesStatusMessage.getJointAngles()[i]);
            }
            armMultiBodyGraphics.get(definition.getSide()).updateFramesRecursively();
            armMultiBodyGraphics.get(definition.getSide()).updateSubtreeGraphics();
         }

         // We probably don't want to recolor the mesh every tick.
         Color color = handPoseJointAnglesStatusMessage.getSolutionQuality() > 1.0 ? badQualityColor : goodQualityColor;
         if (color != currentColor.get(definition.getSide()))
         {
            currentColor.put(definition.getSide(), color);
            for (RigidBodyBasics body : armMultiBodyGraphics.get(definition.getSide()).subtreeIterable())
            {
               if (body instanceof RDXRigidBody rdxRigidBody)
               {
                  if (rdxRigidBody.getVisualGraphicsNode() != null)
                  {
                     for (RDXFrameNodePart part : rdxRigidBody.getVisualGraphicsNode().getParts())
                     {
                        part.getModelInstance().setDiffuseColor(color);
                     }
                  }
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
      jointSpaceControlWrapper.renderImGuiWidget();
      if (!definition.getJointSpaceControl())
      {
         holdPoseInWorldLaterWrapper.renderImGuiWidget();
      }
      parentFrameComboBox.render(syncedRobot.getReferenceFrames().getCommonReferenceFrames());
      ImGui.pushItemWidth(80.0f);
      trajectoryDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   public void render3DPanelImGuiOverlays()
   {
      if (isMouseHovering)
      {
         tooltip.render("%s Action\nIndex: %d\nDescription: %s".formatted(getActionTypeTitle(),
                                                                          state.getActionIndex(),
                                                                          definition.getDescription()));
      }
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (state.getPalmFrame().isChildOfWorld())
      {
         poseGizmo.calculate3DViewPick(input);

         pickResult.reset();
         for (MouseCollidable mouseCollidable : mouseCollidables)
         {
            double collision = mouseCollidable.collide(input.getPickRayInWorld(), collisionShapeFrame.getReferenceFrame());
            if (!Double.isNaN(collision))
               pickResult.addPickCollision(collision);
         }
         if (pickResult.getPickCollisionWasAddedSinceReset())
            input.addPickResult(pickResult);
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (state.getPalmFrame().isChildOfWorld())
      {
         isMouseHovering = input.getClosestPick() == pickResult;

         boolean isClickedOn = isMouseHovering && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
         if (isClickedOn)
         {
            getSelected().set(true);
         }

         poseGizmo.process3DViewInput(input, isMouseHovering);

         tooltip.setInput(input);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (state.getPalmFrame().isChildOfWorld())
      {
         highlightModels.get(definition.getSide()).getRenderables(renderables, pool);
         poseGizmo.getVirtualRenderables(renderables, pool);

         if (state.getIsNextForExecution())
            armMultiBodyGraphics.get(definition.getSide()).getVisualRenderables(renderables, pool);
      }
   }

   @Override
   public String getActionTypeTitle()
   {
      return definition.getSide().getPascalCaseName() + " Hand Pose";
   }

   public ReferenceFrame getReferenceFrame()
   {
      return poseGizmo.getPoseGizmo().getGizmoFrame();
   }

   @Override
   public HandPoseActionState getState()
   {
      return state;
   }

   @Override
   public HandPoseActionDefinition getDefinition()
   {
      return definition;
   }
}