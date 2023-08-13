package us.ihmc.vulkan;

import java.util.stream.IntStream;

public class QueueFamilyIndices
{
   // We use Integer to use null as the empty value
   private Integer graphicsFamily;
   private Integer presentFamily;

   public void setGraphicsFamily(Integer graphicsFamily)
   {
      this.graphicsFamily = graphicsFamily;
   }

   public Integer getGraphicsFamily()
   {
      return graphicsFamily;
   }

   public void setPresentFamily(Integer presentFamily)
   {
      this.presentFamily = presentFamily;
   }

   public Integer getPresentFamily()
   {
      return presentFamily;
   }

   public boolean isComplete()
   {
      return graphicsFamily != null && presentFamily != null;
   }

   public int[] unique()
   {
      return IntStream.of(graphicsFamily, presentFamily).distinct().toArray();
   }

   public int[] array()
   {
      return new int[] {graphicsFamily, presentFamily};
   }
}
