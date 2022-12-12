package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;

public class PlanarRegionFilteredMapUIPanel
{
   private boolean captured = false;

   private ImGuiStoredPropertySetTuner mappingParametersTuner;
   private final RDXPlanarRegionsGraphic mapPlanarRegionsGraphic = new RDXPlanarRegionsGraphic();
   private PlanarRegionMappingManager mappingManager;
   private ImGuiPanel imGuiPanel;
   private final ImBoolean liveModeEnabled = new ImBoolean();

   public PlanarRegionFilteredMapUIPanel(String name, PlanarRegionMappingManager mappingManager)
   {
      imGuiPanel = new ImGuiPanel(name, this::renderImGuiWidgets);
      this.mappingManager = mappingManager;

      // TODO: Connect this to UI. JIRA Ticket HS-330 (https://jira.ihmc.us/browse/HS-330)
      //mappingParametersTuner = new ImGuiStoredPropertySetTuner(mappingManager.getFilteredMap().getParameters().getTitle());
      //mappingParametersTuner.create(mappingManager.getFilteredMap().getParameters());


      mapPlanarRegionsGraphic.generateMeshes(mappingManager.pollMapRegions());
      mapPlanarRegionsGraphic.update();
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.button("Capture"))
      {
         mappingManager.setCaptured(true);
      }

      if (ImGui.button("Load Next Set"))
      {
         mappingManager.nextButtonCallback();
      }

      if (ImGui.checkbox("Enable Live Mode", liveModeEnabled))
      {
         mappingManager.setEnableLiveMode(liveModeEnabled.get());
      }

      if (ImGui.button("Reset map"))
      {
         mappingManager.resetMap();
      }
   }

   public void renderPlanarRegions()
   {
      if (mappingManager.pollIsModified() && mappingManager.hasPlanarRegionsToRender())
      {
         mapPlanarRegionsGraphic.clear();
         mapPlanarRegionsGraphic.generateMeshes(mappingManager.pollMapRegions());
         mapPlanarRegionsGraphic.update();
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      mapPlanarRegionsGraphic.getRenderables(renderables, pool);
   }

   public void setCaptured(boolean captured)
   {
      this.captured = captured;
   }

   public boolean isCaptured()
   {
      return captured;
   }

   public ImGuiPanel getImGuiPanel()
   {
      return imGuiPanel;
   }
}
