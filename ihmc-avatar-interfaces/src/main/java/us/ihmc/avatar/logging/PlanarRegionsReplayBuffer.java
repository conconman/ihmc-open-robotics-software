package us.ihmc.avatar.logging;

import org.apache.commons.io.IOUtils;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.StringReader;
import java.util.*;

public class PlanarRegionsReplayBuffer<T>
{
   private int buffer_length;
   private HashMap<Integer, PlanarRegionsBufferElement<T>> indexBuffer;
   private TreeSet<PlanarRegionsBufferElement<T>> timeBuffer;
   private long firstEverTime = Long.MAX_VALUE;

   private int index = 0;



   private static final Comparator<PlanarRegionsBufferElement> customCompare = (o1, o2) ->
   {
      long compare = o1.getTime() - o2.getTime();
      if (compare > 0)
         return 1;
      else if (compare == 0)
         return 0;
      else
         return -1;
   };

   public void loadFromLog(File planarRegionListLog, Class<?> type) throws IOException
   {
      indexBuffer = new HashMap<>();
      timeBuffer = new TreeSet<>(customCompare);

      Scanner in = new Scanner(planarRegionListLog);
      in.useDelimiter("##\n");

      in.next(); //Skip metadata TODO process metadata
      int index = 0;
      while (in.hasNext())
      {
         in.nextLine(); //Skip past delimiter
         long time = Long.parseLong(in.nextLine());

         final File temp = File.createTempFile("prll", ".tmp");
         temp.deleteOnExit();
         try (FileOutputStream out = new FileOutputStream(temp))
         {
            IOUtils.copy(new StringReader(in.next()), out);
         }

         Object list;

         if (type == FramePlanarRegionsList.class)
            list = PlanarRegionFileTools.importFramePlanarRegionsData(temp);
         else
            list = PlanarRegionFileTools.importPlanarRegionData(temp);

         PlanarRegionsBufferElement planarRegionsBufferElement = new PlanarRegionsBufferElement<>(index, time, list);

         indexBuffer.put(index, planarRegionsBufferElement);
         timeBuffer.add(planarRegionsBufferElement);

         firstEverTime = getStartTime();

         index++;
      }

      if (buffer_length < indexBuffer.size())
         buffer_length = indexBuffer.size();

      if (buffer_length <= 0)
      {
         LogTools.warn("Loaded empty log into TBuffer");
      }
   }

   public PlanarRegionsReplayBuffer(File planarRegionListLog, Class<?> type) throws IOException
   {
      loadFromLog(planarRegionListLog, type);
      buffer_length = indexBuffer.size();
   }

   public PlanarRegionsReplayBuffer()
   {
      this(Integer.MAX_VALUE);
   }

   public PlanarRegionsReplayBuffer(int buffer_length)
   {
      this.buffer_length = buffer_length;
      this.indexBuffer = new HashMap<>();
      this.timeBuffer = new TreeSet<>(customCompare);
   }

   public void expandBuffer(long additionalSize)
   {
      if (additionalSize <= 0)
         return;

      buffer_length += additionalSize;
   }

   public void putAndTick(long time, T list)
   {
      PlanarRegionsBufferElement bufferElement = new PlanarRegionsBufferElement<>(index, time, list);

      indexBuffer.put(index, bufferElement);
      timeBuffer.add(bufferElement);

      if (index > buffer_length)
      {
         indexBuffer.remove(index - buffer_length);
         timeBuffer.remove(timeBuffer.first());
      }

      if (time < firstEverTime)
         firstEverTime = time;

      index++;
   }

   public T get(int index)
   {
      PlanarRegionsBufferElement container = indexBuffer.get(index);
      return container == null ? null : (T) container.getList();
   }

   private PlanarRegionsBufferElement<T> getNearTimeInternal(long time)
   {
      PlanarRegionsBufferElement<T> lookup = new PlanarRegionsBufferElement<>(-1, time, null);
      PlanarRegionsBufferElement<T> lower = timeBuffer.lower(lookup);
      PlanarRegionsBufferElement<T> higher = timeBuffer.higher(lookup);

      PlanarRegionsBufferElement<T> value;

      if (lower == null)
      {
         if (higher == null)
            return null;
         else
            value = higher;
      }
      else if (higher == null)
         value = lower;
      else if (Math.abs(lower.getTime() - time) > Math.abs(higher.getTime() - time))
         value = higher;
      else
         value = lower;

      return value;
   }

   public T getNearTime(long time)
   {
      PlanarRegionsBufferElement<T> container = getNearTimeInternal(time);
      return container != null ? container.getList() : null;
   }

   public long getNextTime(long currentTime)
   {
      if (indexBuffer.size() < 1)
         return -1;

      PlanarRegionsBufferElement<T> container = indexBuffer.get(
            Objects.requireNonNull(getNearTimeInternal(currentTime + 1)).getIndex() + 1); //Increment currentTime by 1 to bias towards higher if tied

      return container == null ? Long.MAX_VALUE : container.getTime();
   }

   public long getPreviousTime(long currentTime)
   {
      if (indexBuffer.size() < 1)
         return -1;

      PlanarRegionsBufferElement container = indexBuffer.get(getNearTimeInternal(currentTime - 1).getIndex() - 1); //Decrement currentTime by 1 to bias towards lower if tied

      return container == null ? 0 : container.getTime();
   }

   public long getCurrentIndex()
   {
      return index;
   }

   public int getBufferLength()
   {
      return buffer_length;
   }

   public long getFirstEverTime()
   {
      return firstEverTime;
   }

   public long getStartTime()
   {
      if (timeBuffer.size() == 0)
         return -1;

      return timeBuffer.first().getTime();
   }

   public long getEndTime()
   {
      if (timeBuffer.size() == 0)
         return -1;

      return timeBuffer.last().getTime();
   }
}
