package us.ihmc.behaviors.activeMapping;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.behaviors.activeMapping.ContinuousWalkingParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface ContinuousWalkingParametersReadOnly extends StoredPropertySetReadOnly
{
   default boolean getContinuousWalkingEnabled()
   {
      return get(continuousWalkingEnabled);
   }

   default boolean getStepPublisherEnabled()
   {
      return get(stepPublisherEnabled);
   }

   default boolean getOverrideEntireQueueEachStep()
   {
      return get(overrideEntireQueueEachStep);
   }

   default int getNumberOfStepsToSend()
   {
      return get(numberOfStepsToSend);
   }

   default double getGoalPoseForwardDistance()
   {
      return get(goalPoseForwardDistance);
   }

   default double getGoalPoseUpDistance()
   {
      return get(goalPoseUpDistance);
   }

   default double getSwingTime()
   {
      return get(swingTime);
   }

   default double getTransferTime()
   {
      return get(transferTime);
   }

   default double getPlanningTimeoutFraction()
   {
      return get(planningTimeoutFraction);
   }

   default double getPlanningReferenceTimeout()
   {
      return get(planningReferenceTimeout);
   }

   default double getPlanningWithoutReferenceTimeout()
   {
      return get(planningWithoutReferenceTimeout);
   }

   default boolean getLogFootstepPlans()
   {
      return get(logFootstepPlans);
   }
}
