package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;

/**
 * Class meant to calculate the preview window over which the model predictive controller operates. It is very unlikely that the MPC will calculate over the
 * full time horizon of steps. That means that some subset over a shorter horizon is needed to be computed for that. This class is used to compute what the
 * contact sequence is over that preview window.
 */
public class PreviewWindowCalculator
{
   protected final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoInteger activeSegment = new YoInteger("activeSegmentInWindow", registry);
   private final YoBoolean activeSegmentChanged = new YoBoolean("activeSegmentChanged", registry);

   private final YoDouble maximumPreviewWindowDuration = new YoDouble("maximumPreviewWindowDuration", registry);
   private final YoInteger maximumPreviewWindowSegments = new YoInteger("maximumPreviewWindowSegments", registry);

   private final YoInteger segmentsInPreviewWindow = new YoInteger("segmentsInPreviewWindow", registry);
   private final YoDouble previewWindowDuration = new YoDouble("previewWindowDuration", registry);

   private final RecyclingArrayList<PreviewWindowSegment> previewWindowContacts = new RecyclingArrayList<>(PreviewWindowSegment::new);
   private final RecyclingArrayList<ContactPlaneProvider> fullContactSet = new RecyclingArrayList<>(ContactPlaneProvider::new);

   private final PreviewWindowSegment trimmedFinalSegment = new PreviewWindowSegment();

   private final ContactSegmentHelper contactSegmentHelper = new ContactSegmentHelper();

   public PreviewWindowCalculator(YoRegistry parentRegistry)
   {
      activeSegment.set(-1);
      this.maximumPreviewWindowDuration.set(0.75);
      this.maximumPreviewWindowSegments.set(3);

      parentRegistry.addChild(registry);
   }

   /**
    * Computes the preview window contacts from the full contact sequence.
    *
    * @param fullContactSequence entire contact sequence. It may start before {@param timeAtStartOfWindow}
    * @param timeAtStartOfWindow time at the start of the preview window
    */
   public void compute(List<ContactPlaneProvider> fullContactSequence, double timeAtStartOfWindow)
   {
      previewWindowContacts.clear();
      double previewWindowLength = computePlanningHorizon(fullContactSequence, timeAtStartOfWindow);

      this.previewWindowDuration.set(previewWindowLength);
      segmentsInPreviewWindow.set(previewWindowContacts.size());
   }

   /**
    * Gets the total duration that the preview window consists of.
    * @return duration of preview window
    */
   public double getPreviewWindowDuration()
   {
      return previewWindowDuration.getDoubleValue();
   }

   private double computePlanningHorizon(List<ContactPlaneProvider> fullContactSequence, double timeAtStartOfWindow)
   {
      int activeSegment = -1;
      activeSegmentChanged.set(false);
      for (int i = 0; i < fullContactSequence.size(); i++)
      {
         TimeIntervalReadOnly timeInterval = fullContactSequence.get(i).getTimeInterval();
         boolean segmentIsValid = timeInterval.intervalContains(timeAtStartOfWindow);
         boolean notAtEndOfSegment = i >= fullContactSequence.size() - 1 || timeAtStartOfWindow < timeInterval.getEndTime();
         if (segmentIsValid && notAtEndOfSegment)
         {
            activeSegment = i;
            break;
         }
      }

      if (this.activeSegment.getIntegerValue() != activeSegment)
      {
         activeSegmentChanged.set(true);
         this.activeSegment.set(activeSegment);
      }

      previewWindowContacts.clear();

      double horizonDuration = -timeAtStartOfWindow;
      for (int i = activeSegment; i < fullContactSequence.size(); i++)
      {
         ContactPlaneProvider contact = fullContactSequence.get(i);

         previewWindowContacts.add().addContactPhaseInSegment(contact, contact.getTimeInterval().getStartTime(), contact.getTimeInterval().getEndTime());
         horizonDuration += contact.getTimeInterval().getDuration();

         if (contact.getContactState().isLoadBearing() && (horizonDuration >= maximumPreviewWindowDuration.getDoubleValue()
                                                           || previewWindowContacts.size() > maximumPreviewWindowSegments.getValue() - 1))
            break;
      }

      double startAlpha = (timeAtStartOfWindow - fullContactSequence.get(activeSegment).getTimeInterval().getStartTime()) / fullContactSequence.get(activeSegment).getTimeInterval().getDuration();
      contactSegmentHelper.cubicInterpolateStartOfSegment(previewWindowContacts.get(0).getContactPhase(0), startAlpha);

      double previewWindowLength = 0.0;
      double flightDuration = 0.0;
      for (int i = 0; i < previewWindowContacts.size() - 1; i++)
      {
         double duration = previewWindowContacts.get(i).getDuration();
         if (previewWindowContacts.get(i).getContactState().isLoadBearing())
            previewWindowLength += duration;
         else
            flightDuration += duration;
      }

      double finalSegmentDuration = maximumPreviewWindowDuration.getDoubleValue() - previewWindowLength;
      PreviewWindowSegment lastSegment = previewWindowContacts.getLast();
      ContactStateBasics<?> lastPhase = lastSegment.getContactPhase(lastSegment.getNumberOfContactPhasesInSegment() - 1);
      double alpha = Math.min(finalSegmentDuration / lastPhase.getTimeInterval().getDuration(), 1.0);
      trimmedFinalSegment.reset();
      if (alpha < 1.0)
      {
         double startTime = lastPhase.getTimeInterval().getStartTime();
         double oldEndTime = lastPhase.getTimeInterval().getEndTime();
         double newEndTime = alpha * lastPhase.getTimeInterval().getDuration() + startTime;
         trimmedFinalSegment.addContactPhaseInSegment(lastPhase, newEndTime, oldEndTime);
         for (int i = 0; i < lastSegment.getNumberOfContacts(); i++)
            trimmedFinalSegment.addContact(lastSegment.getContactPose(i), lastSegment.getContactsInBodyFrame(i));
         contactSegmentHelper.cubicInterpolateStartOfSegment(trimmedFinalSegment.getContactPhase(0), alpha);
         contactSegmentHelper.cubicInterpolateEndOfSegment(lastPhase, alpha);
      }
      previewWindowLength += lastSegment.getDuration();

      fullContactSet.clear();
      for (int i = 0; i < previewWindowContacts.size(); i++)
         setPlaneProviderFromPreviewWindowSegment(fullContactSet.add(), previewWindowContacts.get(i));

      if (alpha < 1.0)
      {
         setPlaneProviderFromPreviewWindowSegment(fullContactSet.add(), trimmedFinalSegment);
         for (int i = previewWindowContacts.size() + activeSegment; i < fullContactSequence.size(); i++)
            fullContactSet.add().set(fullContactSequence.get(i));
      }
      else
      {
         for (int i = previewWindowContacts.size() + activeSegment; i < fullContactSequence.size(); i++)
            fullContactSet.add().set(fullContactSequence.get(i));
      }

      if (!checkContactSequenceIsValid(previewWindowContacts))
         throw new IllegalArgumentException("The preview window is not valid.");
      if (!ContactStateProviderTools.checkContactSequenceIsValid(fullContactSet))
         throw new IllegalArgumentException("The full contact sequence is not valid.");

      return previewWindowLength + flightDuration;
   }

   /**
    * Gets the contact sequence that composes the preview window
    * @return contacts in the preview window
    */
   public List<PreviewWindowSegment> getPlanningWindow()
   {
      return previewWindowContacts;
   }

   /**
    * Gets the full contact sequence. Note that this is likely different than the contact sequence passed in in {@link #computePlanningHorizon(List, double)},
    * as it the preview sequence likely includes only a partial contact. The contact set returned by this function includes breaking that contact sequence into
    * a portion that is included in the preview window, and a portion that is not.
    * @return all the contacts for the entire plan
    */
   public List<ContactPlaneProvider> getFullPlanningSequence()
   {
      return fullContactSet;
   }

   /**
    * Computes whether or not a segment in the {@link #getFullPlanningSequence()} was completed, and then excluded from {@link #getPlanningWindow()}.
    * @return if a segment was just completed
    */
   public boolean activeSegmentChanged()
   {
      return activeSegmentChanged.getBooleanValue();
   }

   public static boolean checkContactSequenceIsValid(List<PreviewWindowSegment> contactStateSequence)
   {
      if (!checkContactSequenceDoesNotEndInFlight(contactStateSequence))
         return false;

      return isTimeSequenceContinuous(contactStateSequence, 1e-2);
   }

   static boolean checkContactSequenceDoesNotEndInFlight(List<PreviewWindowSegment> contactStateSequence)
   {
      return contactStateSequence.get(contactStateSequence.size() - 1).getContactState().isLoadBearing();
   }

   public static boolean isTimeSequenceContinuous(List<PreviewWindowSegment> contactStateSequence, double epsilon)
   {
      for (int index = 0; index < contactStateSequence.size() - 1; index++)
      {
         if (!TimeIntervalTools.areTimeIntervalsConsecutive(contactStateSequence.get(index), contactStateSequence.get(index + 1), epsilon))
            return false;
      }

      return true;
   }

   private static void setPlaneProviderFromPreviewWindowSegment(ContactPlaneProvider planeProviderToPack, PreviewWindowSegment segment)
   {
      planeProviderToPack.reset();
      planeProviderToPack.getTimeInterval().set(segment);

      planeProviderToPack.setStartECMPPosition(segment.getContactPhase(0).getECMPStartPosition());
      planeProviderToPack.setStartECMPVelocity(segment.getContactPhase(0).getECMPStartVelocity());
      int lastId = segment.getNumberOfContactPhasesInSegment() - 1;
      planeProviderToPack.setEndECMPPosition(segment.getContactPhase(lastId).getECMPEndPosition());
      planeProviderToPack.setEndECMPVelocity(segment.getContactPhase(lastId).getECMPEndVelocity());

      for (int i = 0; i < segment.getNumberOfContacts(); i++)
         planeProviderToPack.addContact(segment.getContactPose(i), segment.getContactsInBodyFrame(i));
   }
}
