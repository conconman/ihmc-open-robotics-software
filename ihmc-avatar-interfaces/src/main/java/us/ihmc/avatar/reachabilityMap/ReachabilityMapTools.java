package us.ihmc.avatar.reachabilityMap;

import java.awt.Color;
import java.util.List;

import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.VisualizationSession;

public class ReachabilityMapTools
{
   @Deprecated
   public static Graphics3DObject createBoundingBoxGraphics(FramePoint3DReadOnly min, FramePoint3DReadOnly max)
   {
      double width = 0.01;
      AppearanceDefinition appearance = YoAppearance.LightBlue();
      Graphics3DObject boundingBox = new Graphics3DObject();
      FramePoint3D modifiableMin = new FramePoint3D(min);
      modifiableMin.changeFrame(ReferenceFrame.getWorldFrame());
      FramePoint3D modifiableMax = new FramePoint3D(max);
      modifiableMax.changeFrame(ReferenceFrame.getWorldFrame());
      double x0 = modifiableMin.getX();
      double y0 = modifiableMin.getY();
      double z0 = modifiableMin.getZ();
      double x1 = modifiableMax.getX();
      double y1 = modifiableMax.getY();
      double z1 = modifiableMax.getZ();
      // The three segments originating from min
      boundingBox.addMeshData(MeshDataGenerator.Line(x0, y0, z0, x1, y0, z0, width), appearance);
      boundingBox.addMeshData(MeshDataGenerator.Line(x0, y0, z0, x0, y1, z0, width), appearance);
      boundingBox.addMeshData(MeshDataGenerator.Line(x0, y0, z0, x0, y0, z1, width), appearance);
      // The three segments originating from min
      boundingBox.addMeshData(MeshDataGenerator.Line(x1, y1, z1, x0, y1, z1, width), appearance);
      boundingBox.addMeshData(MeshDataGenerator.Line(x1, y1, z1, x1, y0, z1, width), appearance);
      boundingBox.addMeshData(MeshDataGenerator.Line(x1, y1, z1, x1, y1, z0, width), appearance);

      boundingBox.addMeshData(MeshDataGenerator.Line(x1, y0, z0, x1, y1, z0, width), appearance);
      boundingBox.addMeshData(MeshDataGenerator.Line(x1, y0, z0, x1, y0, z1, width), appearance);

      boundingBox.addMeshData(MeshDataGenerator.Line(x0, y1, z0, x1, y1, z0, width), appearance);
      boundingBox.addMeshData(MeshDataGenerator.Line(x0, y1, z0, x0, y1, z1, width), appearance);

      boundingBox.addMeshData(MeshDataGenerator.Line(x0, y0, z1, x1, y0, z1, width), appearance);
      boundingBox.addMeshData(MeshDataGenerator.Line(x0, y0, z1, x0, y1, z1, width), appearance);

      return boundingBox;
   }

   public static List<VisualDefinition> createBoundingBoxVisuals(FramePoint3DReadOnly min, FramePoint3DReadOnly max)
   {
      double width = 0.01;
      ColorDefinition color = ColorDefinitions.LightBlue();
      VisualDefinitionFactory boundingBox = new VisualDefinitionFactory();
      FramePoint3D modifiableMin = new FramePoint3D(min);
      modifiableMin.changeFrame(ReferenceFrame.getWorldFrame());
      FramePoint3D modifiableMax = new FramePoint3D(max);
      modifiableMax.changeFrame(ReferenceFrame.getWorldFrame());
      double x0 = modifiableMin.getX();
      double y0 = modifiableMin.getY();
      double z0 = modifiableMin.getZ();
      double x1 = modifiableMax.getX();
      double y1 = modifiableMax.getY();
      double z1 = modifiableMax.getZ();
      // The three segments originating from min
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y0, z0, x1, y0, z0, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y0, z0, x0, y1, z0, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y0, z0, x0, y0, z1, width), color);
      // The three segments originating from min
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x1, y1, z1, x0, y1, z1, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x1, y1, z1, x1, y0, z1, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x1, y1, z1, x1, y1, z0, width), color);

      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x1, y0, z0, x1, y1, z0, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x1, y0, z0, x1, y0, z1, width), color);

      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y1, z0, x1, y1, z0, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y1, z0, x0, y1, z1, width), color);

      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y0, z1, x1, y0, z1, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y0, z1, x0, y1, z1, width), color);

      return boundingBox.getVisualDefinitions();
   }

   @Deprecated
   public static Graphics3DObject createReachibilityColorScale()
   {
      Graphics3DObject voxelViz = new Graphics3DObject();
      double maxReachability = 0.7;
      double resolution = 0.1;
      voxelViz.translate(-1.0, -1.0, 0.0);

      for (double z = 0; z <= maxReachability; z += maxReachability * resolution)
      {
         AppearanceDefinition appearance = YoAppearance.RGBColorFromHex(Color.HSBtoRGB((float) z, 1.0f, 1.0f));
         voxelViz.translate(0.0, 0.0, resolution);
         voxelViz.addSphere(0.025, appearance);
      }

      return voxelViz;
   }

   public static List<VisualDefinition> createReachibilityColorScaleVisuals()
   {
      VisualDefinitionFactory voxelViz = new VisualDefinitionFactory();
      double maxReachability = 0.7;
      double resolution = 0.1;
      voxelViz.appendTranslation(-1.0, -1.0, 0.0);

      for (double z = 0; z <= maxReachability; z += maxReachability * resolution)
      {
         ColorDefinition color = ColorDefinitions.hsb(z * 360.0, 1.0, 1.0);
         voxelViz.appendTranslation(0.0, 0.0, resolution);
         voxelViz.addSphere(0.025, color);
      }

      return voxelViz.getVisualDefinitions();
   }

   public static void loadVisualizeReachabilityMap(String robotName, RobotDefinition robotDefinition, FullHumanoidRobotModel fullRobotModel)
   {
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      long startTime = System.nanoTime();
      System.out.println("Loading reachability map");
      ReachabilityMapFileLoader reachabilityMapFileLoader = new ReachabilityMapFileLoader(robotName, fullRobotModel.getElevator(), referenceFrames);
   
      System.out.println("Done loading reachability map. Took: " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime) + " seconds.");
   
      Voxel3DGrid grid = reachabilityMapFileLoader.getLoadedGrid();
      SphereVoxelShape sphereVoxelShape = grid.getSphereVoxelShape();
   
      List<VisualDefinition> allVisuals = createReachibilityColorScaleVisuals();
   
      int numberOfVoxel = grid.getNumberOfVoxelsPerDimension();
      grid.getReferenceFrame().update();
      System.out.println(grid.getReferenceFrame().getTransformToWorldFrame());
   
      for (int xIndex = 0; xIndex < numberOfVoxel; xIndex++)
      {
         for (int yIndex = 0; yIndex < numberOfVoxel; yIndex++)
         {
            for (int zIndex = 0; zIndex < numberOfVoxel; zIndex++)
            {
               double reachabilityValue = grid.getD(xIndex, yIndex, zIndex);
   
               if (reachabilityValue > 0.001)
               {
                  Voxel3DData voxel = grid.getVoxel(xIndex, yIndex, zIndex);
                  System.out.println("xIndex: " + xIndex + ", yIndex: " + yIndex + ", zIndex: " + zIndex + ", voxel location: " + voxel.getPosition());
                  allVisuals.add(sphereVoxelShape.createVisual(voxel.getPosition(), 0.25, reachabilityValue));
               }
            }
         }
      }
   
      VisualizationSession visualizationSession = new VisualizationSession(robotName + " Reachability Map Visualizer");
      visualizationSession.addRobot(robotDefinition);
      SessionVisualizerControls guiControls = SessionVisualizer.startSessionVisualizer(visualizationSession);
      guiControls.addStaticVisuals(allVisuals);
      guiControls.waitUntilDown();
   }
}
