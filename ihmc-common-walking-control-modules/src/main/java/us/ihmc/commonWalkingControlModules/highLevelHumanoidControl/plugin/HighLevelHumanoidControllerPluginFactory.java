package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;

public interface HighLevelHumanoidControllerPluginFactory
{
   HighLevelHumanoidControllerPlugin buildPlugin(HighLevelControllerFactoryHelper controllerFactoryHelper);
}
