package us.ihmc.gdx.ui.yo;

import imgui.extension.implot.ImPlot;
import us.ihmc.gdx.simulation.scs2.GDXYoManager;
import us.ihmc.gdx.ui.tools.ImPlotTools;
import us.ihmc.scs2.sharedMemory.BufferSample;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.function.Consumer;

public class ImPlotYoBufferLongPlotLine extends ImPlotYoBufferPlotLineBasics
{
   private final YoLong yoLong;
   private LinkedYoVariable<YoLong> linkedYoLongVariable;
   private Integer[] xValues = ImPlotTools.createIndex(1);
   private Long[] plotData = ImPlotTools.newZeroFilledLongBuffer(1);

   public ImPlotYoBufferLongPlotLine(YoLong yoLong, Consumer<YoVariable> removeSelf)
   {
      super(yoLong, "0", removeSelf);
      this.yoLong = yoLong;
   }

   @Override
   public void setupLinkedVariable(GDXYoManager yoManager)
   {
      if (linkedYoLongVariable == null)
      {
         linkedYoLongVariable = (LinkedYoVariable<YoLong>) yoManager.newLinkedYoVariable(yoLong);
         linkedYoLongVariable.addUser(this);
      }
   }

   @Override
   public void update()
   {
      if (linkedYoLongVariable != null)
      {
         linkedYoLongVariable.pull();

         if (linkedYoLongVariable.isRequestedBufferSampleAvailable())
         {
            BufferSample<long[]> bufferSample = linkedYoLongVariable.pollRequestedBufferSample();
            long[] buffer = bufferSample.getSample();
            int sampleLength = bufferSample.getSampleLength();
            if (plotData.length != sampleLength)
            {
               xValues = ImPlotTools.createIndex(sampleLength);
               plotData = ImPlotTools.newZeroFilledLongBuffer(sampleLength);
            }
            for (int i = 0; i < bufferSample.getBufferProperties().getActiveBufferLength(); i++)
            {
               plotData[i] = buffer[i];
            }
         }

         linkedYoLongVariable.requestEntireBuffer();
      }
   }

   @Override
   protected void plot(String labelID)
   {
      int offset = 0; // This is believed to be the index in the array we are passing in which implot will start reading
      ImPlot.plotLine(labelID, xValues, plotData, offset);
   }

   @Override
   public String getValueString(int bufferIndex)
   {
      return String.valueOf(plotData[bufferIndex]);
   }
}
