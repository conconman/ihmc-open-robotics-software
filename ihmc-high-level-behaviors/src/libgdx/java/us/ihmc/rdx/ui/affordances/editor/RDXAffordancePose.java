package us.ihmc.rdx.ui.affordances.editor;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.interactable.RDXInteractableSakeGripper;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class RDXAffordancePose
{
   private FramePose3D pose = new FramePose3D();
   private boolean isPoseSet = false;
   private final RDXReferenceFrameGraphic frameGraphic;
   private final PoseReferenceFrame frame;
   private HandConfiguration handConfiguration;
   public final PoseReferenceFrame affordanceFrame;
   public final RDXInteractableSakeGripper interactableHand;
   private final FramePose3D handPose;
   private final RigidBodyTransform handTransformToWorld;
   private RDXActiveAffordanceMenu[] activeMenu;
   private final RDXActiveAffordanceMenu menu;
   public boolean changedColor = false;

   public RDXAffordancePose(RDXInteractableSakeGripper interactableHand,
                            RigidBodyTransform handTransformToWorld,
                            FramePose3D handPose,
                            PoseReferenceFrame affordanceFrame,
                            RDXActiveAffordanceMenu[] activeMenu,
                            Color color)
   {
      this.interactableHand = interactableHand;
      this.handPose = handPose;
      this.handTransformToWorld = handTransformToWorld;
      this.activeMenu = activeMenu;
      this.menu = activeMenu[0];
      this.affordanceFrame = affordanceFrame;
      frame = new PoseReferenceFrame("handFrame", affordanceFrame);
      frameGraphic = new RDXReferenceFrameGraphic(0.1, color);
   }

   public void update()
   {
      pose = new FramePose3D(frame);
      pose.changeFrame(ReferenceFrame.getWorldFrame());
      frameGraphic.updateFromFramePose(pose);
   }

   public void renderImGuiWidgets(ImGuiUniqueLabelMap labels, String id)
   {
      if (ImGui.button(labels.get("SET") + "##" + id))
      {
         isPoseSet = true;
         frame.setPoseAndUpdate(handPose);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("CLEAR") + "##" + id))
      {
         isPoseSet = false;
         handConfiguration = null;
      }

      if (isPoseSet)
      {
         if(activeMenu[0].equals(this.menu))
         {
            changedColor = true;
            ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 1.0f, 0.0f, 1.0f);
         }
         if (ImGui.button(labels.get("Grasp Frame") + "##" + id))
         {
            activeMenu[0] = this.menu;
            if (isPoseSet)
               handTransformToWorld.set(pose);  // move hand to pregrasp point
            if (handConfiguration != null)
               interactableHand.setGripperToConfiguration(handConfiguration);
         }
         if (changedColor)
         {
            ImGui.popStyleColor();
            changedColor = false;
         }
      }

      ImGui.text("Hand Configuration: " + (handConfiguration == null ? "" : handConfiguration.toString()));
      ImGui.sameLine();
      if (ImGui.button(labels.get("SET") + "##hand" + id) && activeMenu[0].equals(this.menu))
      {
         handConfiguration = interactableHand.getConfiguration();
      }
   }

   public void reset()
   {
      isPoseSet = false;
      handConfiguration = null;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isPoseSet)
         frameGraphic.getRenderables(renderables, pool);
   }

   public PoseReferenceFrame getFrame()
   {
      return frame;
   }

   public FramePose3D getPose()
   {
      return pose;
   }

   public HandConfiguration getHandConfiguration()
   {
      return handConfiguration;
   }
}
