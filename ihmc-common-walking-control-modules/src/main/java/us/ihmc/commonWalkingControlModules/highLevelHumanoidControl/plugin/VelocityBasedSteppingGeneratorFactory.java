package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.DirectionalControlInputMessage;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.DirectionalControlMessenger;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class VelocityBasedSteppingGeneratorFactory implements SteppingPluginFactory
{
   private final RequiredFactoryField<YoRegistry> registryField = new RequiredFactoryField<>("registry");
   private final OptionalFactoryField<StepGeneratorCommandInputManager> csgCommandInputManagerField = new OptionalFactoryField<>("csgCommandInputManagerField");
   private final OptionalFactoryField<VelocityBasedSteppingParameters> inputParametersField = new OptionalFactoryField<>("inputParametersField");

   public void setRegistry()
   {
      setRegistry(ComponentBasedFootstepDataMessageGenerator.class.getSimpleName());
   }

   public void setRegistry(String name)
   {
      registryField.set(new YoRegistry(name));
   }

   public void setInputParameters(VelocityBasedSteppingParameters parameters)
   {
      inputParametersField.set(parameters);
   }

   public StepGeneratorCommandInputManager setStepGeneratorCommandInputManager()
   {
      StepGeneratorCommandInputManager csgCommandInputManager = new StepGeneratorCommandInputManager();
      setStepGeneratorCommandInputManager(csgCommandInputManager);
      return csgCommandInputManager;
   }

   public void setStepGeneratorCommandInputManager(StepGeneratorCommandInputManager commandInputManager)
   {
      this.csgCommandInputManagerField.set(commandInputManager);
   }

   @Override
   public void setFootStepAdjustment(FootstepAdjustment footstepAdjustment)
   {
   }

   @Override
   public StepGeneratorCommandInputManager getStepGeneratorCommandInputManager()
   {
      if (csgCommandInputManagerField.hasValue())
         return csgCommandInputManagerField.get();
      else
         return setStepGeneratorCommandInputManager();
   }

   @Override
   public VelocityBasedSteppingGenerator buildPlugin(CommonHumanoidReferenceFrames referenceFrames,
                                                     double updateDT,
                                                     WalkingControllerParameters walkingControllerParameters,
                                                     StatusMessageOutputManager walkingStatusMessageOutputManager,
                                                     CommandInputManager walkingCommandInputManager,
                                                     YoGraphicsListRegistry yoGraphicsListRegistry,
                                                     SideDependentList<? extends ContactableBody> contactableFeet,
                                                     DoubleProvider timeProvider)
   {
      if (!registryField.hasBeenSet())
         setRegistry();

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      VelocityBasedSteppingGenerator fastWalkingJoystickPlugin = new VelocityBasedSteppingGenerator();

      fastWalkingJoystickPlugin.setHighLevelStateChangeStatusListener(walkingStatusMessageOutputManager);

      fastWalkingJoystickPlugin.setDirectionalControlMessenger(new DirectionalControlMessenger()
      {
         private final DirectionalControlInputMessage message = new DirectionalControlInputMessage();

         @Override
         public void submitDirectionalControlRequest(double desiredXVelocity, double desiredYVelocity, double desiredTurningSpeed)
         {
            message.setForward(desiredXVelocity);
            message.setRight(-desiredYVelocity);
            message.setClockwise(desiredTurningSpeed);

            walkingCommandInputManager.submitMessage(message);
         }
      });
      StepGeneratorCommandInputManager commandInputManager = csgCommandInputManagerField.get();
      fastWalkingJoystickPlugin.setDesiredVelocityProvider(commandInputManager.createDesiredVelocityProvider());
      fastWalkingJoystickPlugin.setDesiredTurningVelocityProvider(commandInputManager.createDesiredTurningVelocityProvider());
      fastWalkingJoystickPlugin.setWalkInputProvider(commandInputManager.createWalkInputProvider());
      if (inputParametersField.hasValue())
         fastWalkingJoystickPlugin.setInputParameters(inputParametersField.get());
      walkingStatusMessageOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class,
                                                                    commandInputManager::setHighLevelStateChangeStatusMessage);

      FactoryTools.disposeFactory(this);

      return fastWalkingJoystickPlugin;
   }
}
