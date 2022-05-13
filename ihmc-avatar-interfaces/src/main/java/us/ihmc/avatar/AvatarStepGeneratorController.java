package us.ihmc.avatar;

import us.ihmc.avatar.stepAdjustment.PlanarRegionFootstepSnapper;
import us.ihmc.avatar.stepAdjustment.PlanarRegionStepConstraintCalculator;
import us.ihmc.avatar.stepAdjustment.PlanarRegionsFilterForStepping;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AvatarStepGeneratorController implements RobotController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ContinuousStepGenerator continuousStepGenerator;
   private final CommandInputManager commandInputManager;
   private final DoubleProvider timeProvider;

   private final PlanarRegionsFilterForStepping planarRegionsFilterForStepping = new PlanarRegionsFilterForStepping();
   private final PlanarRegionFootstepSnapper planarRegionFootstepSnapper;
   private final PlanarRegionStepConstraintCalculator stepConstraintCalculator;

   public AvatarStepGeneratorController(ContinuousStepGenerator continuousStepGenerator,
                                        CommandInputManager commandInputManager,
                                        SteppingParameters steppingParameters,
                                        DoubleProvider timeProvider)
   {
      this.continuousStepGenerator = continuousStepGenerator;
      this.commandInputManager = commandInputManager;
      this.timeProvider = timeProvider;

      planarRegionFootstepSnapper = new PlanarRegionFootstepSnapper(continuousStepGenerator,
                                                                    steppingParameters,
                                                                    registry);
      stepConstraintCalculator = new PlanarRegionStepConstraintCalculator();

      continuousStepGenerator.setFootstepAdjustment(planarRegionFootstepSnapper);
      continuousStepGenerator.setStepConstraintRegionCalculator(stepConstraintCalculator);
   }

   @Override
   public void doControl()
   {
      consumePlanarRegions();

      continuousStepGenerator.update(timeProvider.getValue());
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   private void consumePlanarRegions()
   {
      if (commandInputManager != null)
      {
         if (commandInputManager.isNewCommandAvailable(PlanarRegionsListCommand.class))
         {
            PlanarRegionsListCommand commands = commandInputManager.pollNewestCommand(PlanarRegionsListCommand.class);

            planarRegionsFilterForStepping.setPlanarRegions(commands);

            planarRegionFootstepSnapper.setPlanarRegions(planarRegionsFilterForStepping.getPlanarRegions());
            stepConstraintCalculator.setPlanarRegions(planarRegionsFilterForStepping.getPlanarRegions());
         }

         commandInputManager.clearCommands(PlanarRegionsListCommand.class);
      }
   }
}
