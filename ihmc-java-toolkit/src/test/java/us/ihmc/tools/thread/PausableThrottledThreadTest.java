package us.ihmc.tools.thread;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.robotics.TestTools;
import us.ihmc.tools.time.FrequencyCalculator;
import us.ihmc.tools.time.FrequencyStatisticPrinter;

import static org.junit.jupiter.api.Assertions.*;

public class PausableThrottledThreadTest
{
   private static final String name = "TestPausableThrottledThread";
   private FrequencyStatisticPrinter frequencyStatisticPrinter;
   private FrequencyCalculator frequencyCalculator;

   @Test
   public void test5HzFrequency()
   {
      double hertz = 5.0;

      frequencyStatisticPrinter = new FrequencyStatisticPrinter();
      frequencyCalculator = new FrequencyCalculator();

      PausableThrottledThread thread = new PausableThrottledThread(name, hertz, () ->
      {
         frequencyStatisticPrinter.ping();
         frequencyCalculator.ping();
      });
      thread.start();

      try
      {
         Thread.sleep(5000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }

      frequencyStatisticPrinter.destroy();
      TestTools.assertEpsilonEquals(hertz, frequencyCalculator.getFrequency(), 0.5, "Frequency not correct");
   }

   @Test
   public void test10HzFrequency()
   {
      double hertz = 10.0;

      frequencyStatisticPrinter = new FrequencyStatisticPrinter();
      frequencyCalculator = new FrequencyCalculator();

      PausableThrottledThread thread = new PausableThrottledThread(name, hertz, () ->
      {
         frequencyStatisticPrinter.ping();
         frequencyCalculator.ping();
      });
      thread.start();

      try
      {
         Thread.sleep(5000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }

      frequencyStatisticPrinter.destroy();
      TestTools.assertEpsilonEquals(hertz, frequencyCalculator.getFrequency(), 0.5, "Frequency not correct");
      thread.stop();
   }

   @Test
   public void testStartStopStart()
   {
      double hertz = 10.0;

      PausableThrottledThread thread = new PausableThrottledThread(name, hertz, () ->
      {
         for (int i = 0; i < 25; ++i)
         {
            Thread.sleep(i);
         }
      });

      thread.start();
      assertTrue(thread.isRunning());
      assertTrue(thread.getThread().isAlive());

      thread.stop();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isRunning());
      assertFalse(thread.getThread().isAlive());

      thread.start();
      assertTrue(thread.isRunning());
      assertTrue(thread.getThread().isAlive());

      thread.stop();
   }

   // TODO: Figure out how to test this case
   @Test
   public void testExceptionHandling()
   {
      double hertz = 10.0;

      PausableThrottledThread thread = new PausableThrottledThread(name, hertz, DefaultExceptionHandler.RUNTIME_EXCEPTION, () ->
      {
         throw new NullPointerException("This is a test");
      });

      try
      {
         thread.start();
         Thread.sleep(1000);
      }
      catch (Exception e)
      {
         // Got an exception. This is good!
         assertTrue(true);
      }
      finally
      {
         assertFalse(thread.isRunning());
         assertFalse(thread.getThread().isAlive());
      }
   }
}
