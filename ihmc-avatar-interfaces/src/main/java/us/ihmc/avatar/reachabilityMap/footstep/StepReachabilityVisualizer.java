package us.ihmc.avatar.reachabilityMap.footstep;

import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityLatticePoint;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.log.LogTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

import java.awt.*;
import java.util.Map;

public class StepReachabilityVisualizer
{
   public StepReachabilityVisualizer(StepReachabilityData stepReachabilityData)
   {
      // Set up SCS and coordinate object
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, 16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(parameters);
      Graphics3DObject coordinate = new Graphics3DObject();
      coordinate.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(coordinate);
      scs.setGroundVisible(false);
      scs.setCameraFix(0.0, 0.0, 1.0);
      scs.setCameraPosition(8.0, 0.0, 3.0);
      scs.startOnAThread();
      double yawDivisions = stepReachabilityData.getYawDivisions();

      for (StepReachabilityLatticePoint latticePoint : stepReachabilityData.getLegReachabilityMap().keySet())
      {
         // Represent footpose as sphere, yaw as z-axis translation
         Graphics3DObject validStep = new Graphics3DObject();
         validStep.translate(latticePoint.getXIndex(), latticePoint.getYIndex(), latticePoint.getYawIndex()/(1/yawDivisions));

         // Reachability for this footpose indicated by green/red color
         double reachabilityValue = stepReachabilityData.getLegReachabilityMap().get(latticePoint);
         if (reachabilityValue > 40) reachabilityValue = 40;
         LogTools.info("Reachability value: " + reachabilityValue);
         AppearanceDefinition appearance = YoAppearance.RGBColor(reachabilityValue/40, (40-reachabilityValue)/40, 0);
         validStep.addSphere(yawDivisions, appearance);

         scs.addStaticLinkGraphics(validStep);
      }
   }
}
