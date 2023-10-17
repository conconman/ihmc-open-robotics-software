package us.ihmc.sensorProcessing.frames;

public enum CommonReferenceFrameIds
{
   NONE(-1),
   MIDFEET_ZUP_FRAME(-100),
   MIDFEET_ZUP_GROUND_FRAME(-100),
   PELVIS_ZUP_FRAME(-102),
   PELVIS_FRAME(-103),
   CHEST_FRAME(-104),
   CENTER_OF_MASS_FRAME(-105),
   LEFT_SOLE_FRAME(-106),
   RIGHT_SOLE_FRAME(-107);

   private final long hashId;

   CommonReferenceFrameIds(long hashId)
   {
      this.hashId = hashId;
   }

   public long getHashId()
   {
      return hashId;
   }
}
