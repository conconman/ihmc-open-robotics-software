package us.ihmc.sensorProcessing.heightMap;

import controller_msgs.msg.dds.HeightMapMessage;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.log.LogTools;

import java.util.Arrays;

public class HeightMapData
{
   private final TIntArrayList occupiedCells = new TIntArrayList();
   private final double[] heights;

   private final int minMaxIndexXY;
   private final int cellsPerAxis;
   private final double gridResolutionXY;
   private final double gridSizeXY;

   public HeightMapData(double gridResolutionXY, double gridSizeXY)
   {
      this.gridResolutionXY = gridResolutionXY;
      this.gridSizeXY = gridSizeXY;
      this.minMaxIndexXY = HeightMapTools.toIndex(gridSizeXY, gridResolutionXY, 0);
      cellsPerAxis = (2 * minMaxIndexXY + 1);
      this.heights = new double[cellsPerAxis * cellsPerAxis];

      reset();
   }

   public HeightMapData(HeightMapMessage heightMapMessage)
   {
      this(heightMapMessage.getXyResolution(), heightMapMessage.getGridSizeXy());

      for (int i = 0; i < heightMapMessage.getHeights().size(); i++)
      {
         double height = heightMapMessage.getHeights().get(i);
         int xyIndex = toXYIndex(heightMapMessage.getXCells().get(i), heightMapMessage.getYCells().get(i));
         heights[xyIndex] = height;
         occupiedCells.add(xyIndex);
      }
   }

   public void reset()
   {
      occupiedCells.clear();
      Arrays.fill(heights, Double.NaN);
   }

   public double getGridResolutionXY()
   {
      return gridResolutionXY;
   }

   public double getGridSizeXY()
   {
      return gridSizeXY;
   }

   public int getNumberOfCells()
   {
      return occupiedCells.size();
   }

   public double getHeight(int i)
   {
      return heights[occupiedCells.get(i)];
   }

   /**
    * Returns height at the given (x,y) position, or NaN if there is no height at the given point
    */
   public double getHeightAt(double x, double y)
   {
      int xIndex = HeightMapTools.toIndex(x, gridResolutionXY, minMaxIndexXY);
      int yIndex = HeightMapTools.toIndex(y, gridResolutionXY, minMaxIndexXY);

      if (xIndex < 0 || yIndex < 0 || xIndex > minMaxIndexXY || yIndex > minMaxIndexXY)
      {
         LogTools.error("Invalid index for point (" + x + ", " + y + "). Indices: (" + xIndex + ", " + yIndex + ")");
         return Double.NaN;
      }

      return heights[toXYIndex(xIndex, yIndex)];
   }

   public void setHeightAt(double x, double y, double z)
   {
      int xIndex = HeightMapTools.toIndex(x, gridResolutionXY, minMaxIndexXY);
      int yIndex = HeightMapTools.toIndex(y, gridResolutionXY, minMaxIndexXY);

      if (xIndex < 0 || yIndex < 0 || xIndex > cellsPerAxis || yIndex > cellsPerAxis)
      {
         LogTools.error("Invalid index for point (" + x + ", " + y + "). Indices: (" + xIndex + ", " + yIndex + ")");
         return;
      }

      int xyIndex = toXYIndex(xIndex, yIndex);
      if (Double.isNaN(heights[xyIndex]))
      {
         occupiedCells.add(xyIndex);
      }

      heights[xyIndex] = z;
   }

   public double getHeightAt(int xIndex, int yIndex)
   {
      return heights[toXYIndex(xIndex, yIndex)];
   }

   public int getMinMaxIndexXY()
   {
      return minMaxIndexXY;
   }

   /* Maps xy indices to single value */
   private int toXYIndex(int xIndex, int yIndex)
   {
      return xIndex + (2 * minMaxIndexXY + 1) * yIndex;
   }

   private int xIndex(int xyIndex)
   {
      return xyIndex % (2 * minMaxIndexXY + 1);
   }

   private int yIndex(int xyIndex)
   {
      return xyIndex / (2 * minMaxIndexXY + 1);
   }
}
