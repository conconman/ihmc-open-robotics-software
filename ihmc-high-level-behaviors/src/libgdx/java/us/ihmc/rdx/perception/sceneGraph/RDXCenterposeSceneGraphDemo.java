package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.avatar.colorVision.CenterposeSceneGraphOnRobotProcess;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.visualizers.RDXGlobalVisualizersPanel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.Throttler;

public class RDXCenterposeSceneGraphDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private ROS2Node ros2Node;
   private ROS2Helper ros2Helper;
   private RDXGlobalVisualizersPanel globalVisualizersPanel;
   private CenterposeSceneGraphOnRobotProcess centerposeProcess;
   private ROS2SceneGraph onRobotSceneGraph;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private RDXSceneGraphUI sceneGraphUI;
   private final Throttler perceptionThottler = new Throttler().setFrequency(30.0);

   public RDXCenterposeSceneGraphDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "centerpose_scene_graph_demo");
            ros2Helper = new ROS2Helper(ros2Node);

            globalVisualizersPanel = new RDXGlobalVisualizersPanel();
            baseUI.getImGuiPanelManager().addPanel(globalVisualizersPanel);
            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersPanel);

            onRobotSceneGraph = new ROS2SceneGraph(ros2Helper);
            centerposeProcess = new CenterposeSceneGraphOnRobotProcess(ros2Helper);

            referenceFrameLibrary = new ReferenceFrameLibrary();
            sceneGraphUI = new RDXSceneGraphUI(ros2Helper, baseUI.getPrimary3DPanel(), referenceFrameLibrary);
            referenceFrameLibrary.addDynamicCollection(sceneGraphUI.getSceneGraph().asNewDynamicReferenceFrameCollection());
            baseUI.getPrimaryScene().addRenderableProvider(sceneGraphUI::getRenderables);
            baseUI.getImGuiPanelManager().addPanel(sceneGraphUI.getPanel());

            globalVisualizersPanel.create();
         }

         @Override
         public void render()
         {
            boolean runPerception = perceptionThottler.run();

            if (runPerception)
            {
               onRobotSceneGraph.updateSubscription();
               centerposeProcess.updateSceneGraph(onRobotSceneGraph);
               onRobotSceneGraph.updateOnRobotOnly(ReferenceFrame.getWorldFrame());
               onRobotSceneGraph.updatePublication();
            }

            sceneGraphUI.update();

            globalVisualizersPanel.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            globalVisualizersPanel.destroy();
            baseUI.dispose();
            ros2Node.destroy();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXCenterposeSceneGraphDemo();
   }
}
