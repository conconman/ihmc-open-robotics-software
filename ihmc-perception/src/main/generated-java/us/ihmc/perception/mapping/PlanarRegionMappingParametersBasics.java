package us.ihmc.perception.mapping;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface PlanarRegionMappingParametersBasics extends PlanarRegionMappingParametersReadOnly, StoredPropertySetBasics
{
   default void setUpdateAlphaTowardsMatch(double updateAlphaTowardsMatch)
   {
      set(PlanarRegionMappingParameters.updateAlphaTowardsMatch, updateAlphaTowardsMatch);
   }

   default void setAngleThresholdBetweenNormals(double angleThresholdBetweenNormals)
   {
      set(PlanarRegionMappingParameters.angleThresholdBetweenNormals, angleThresholdBetweenNormals);
   }

   default void setOrthogonalDistanceThreshold(double orthogonalDistanceThreshold)
   {
      set(PlanarRegionMappingParameters.orthogonalDistanceThreshold, orthogonalDistanceThreshold);
   }

   default void setMaxInterRegionDistance(double maxInterRegionDistance)
   {
      set(PlanarRegionMappingParameters.maxInterRegionDistance, maxInterRegionDistance);
   }
}