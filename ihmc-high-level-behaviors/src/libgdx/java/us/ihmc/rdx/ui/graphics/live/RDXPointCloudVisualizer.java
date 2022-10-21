package us.ihmc.rdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;

import java.nio.FloatBuffer;
import java.util.function.Function;

public class RDXPointCloudVisualizer extends RDXVisualizer
{
   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiPlot segmentIndexPlot = new ImGuiPlot("Segment", 1000, 230, 20);
   private final ImFloat pointSize = new ImFloat(0.01f);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final int pointsPerSegment;
   private final int numberOfSegments;
   private int totalNumberOfPoints;
   private final Color color = new Color();
   private final String topicName;
   private int latestSegmentIndex;

<<<<<<< Updated upstream:ihmc-high-level-behaviors/src/libgdx/java/us/ihmc/rdx/ui/graphics/live/RDXPointCloudVisualizer.java
   public RDXPointCloudVisualizer(String title, String topicName, int pointsPerSegment, int numberOfSegments)
   {
      super(title + " (ROS 2)");
      this.pointsPerSegment = pointsPerSegment;
      this.numberOfSegments = numberOfSegments;
      totalNumberOfPoints = pointsPerSegment * numberOfSegments;
      this.topicName = topicName;
      frequencyPlot = plot;
   }

   @Override
   public void create()
   {
      super.create();
      pointCloudRenderer.create(pointsPerSegment, numberOfSegments);
      pointCloudRenderer.getVertexBuffer().limit(3256320);
   }

   @Override
   public void update()
   {
      super.update();
//      updateMeshFastest();
   }

   public void updateMeshFastest()
   {
      pointCloudRenderer.updateMeshFastest(totalNumberOfPoints);
   }

   public void updateMeshFastest(Function<FloatBuffer, Integer> bufferConsumer)
   {
      pointCloudRenderer.updateMeshFastest(bufferConsumer);
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topicName);
      ImGui.sameLine();
      ImGui.pushItemWidth(30.0f);
      ImGui.dragFloat(labels.get("Size"), pointSize.getData(), 0.001f, 0.0005f, 0.1f);
      ImGui.popItemWidth();
      frequencyPlot.renderImGuiWidgets();
      segmentIndexPlot.render(latestSegmentIndex);
   }

   public FloatBuffer getVertexBuffer()
   {
      return pointCloudRenderer.getVertexBuffer();
   }

   public void updatePointCloud(PointCloud pointCloud)
   {
//       = pointCloud.getData();
      for (int i = 0; i < 50; ++i)
      {
         System.out.println(pointCloud.getData()[i]);
      }
      pointCloudRenderer.getVertexBuffer().put(pointCloud.getData());

   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isActive())
         pointCloudRenderer.getRenderables(renderables, pool);
   }

   public void setLatestSegmentIndex(int latestSegmentIndex)
   {
      this.latestSegmentIndex = latestSegmentIndex;
   }

   public void setTotalNumberOfPoints(int totalNumberOfPoints)
   {
      this.totalNumberOfPoints = totalNumberOfPoints;
   }
}
