package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.*;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.ColoringType;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.DisplayType;
import us.ihmc.robotEnvironmentAwareness.ui.properties.SurfaceNormalFilterParametersProperty;

public class SurfaceNormalFilterAnchorPaneController extends REABasicUIController
{

   @FXML
   private Slider surfaceNormalLowerBoundSlider;
   @FXML
   private Slider surfaceNormalUpperBoundSlider;
   @FXML
   private ToggleButton enableSurfaceNormalButton;

   private final SurfaceNormalFilterParametersProperty surfaceNormalFilterParametersProperty = new SurfaceNormalFilterParametersProperty(this,
                                                                                                                                         "surfaceNormalFilterParametersProperty");

   private final PropertyToMessageTypeConverter<Integer, Number> numberToIntegerConverter = new PropertyToMessageTypeConverter<Integer, Number>()
   {
      @Override
      public Integer convert(Number propertyValue)
      {
         return propertyValue.intValue();
      }

      @Override
      public Number interpret(Integer newValue)
      {
         return new Double(newValue.doubleValue());
      }
   };

   private final PropertyToMessageTypeConverter<Double, Number> numberToDoubleConverter = new PropertyToMessageTypeConverter<Double, Number>()
   {
      @Override
      public Double convert(Number propertyValue)
      {
         return propertyValue.doubleValue();
      }

      @Override
      public Number interpret(Double newValue)
      {
         return new Double(newValue.doubleValue());
      }
   };

   public SurfaceNormalFilterAnchorPaneController()
   {
   }

   public void setupControls()
   {

      surfaceNormalUpperBoundSlider.setLabelFormatter(StringConverterTools.radiansToRoundedDegrees());
      surfaceNormalLowerBoundSlider.setLabelFormatter(StringConverterTools.radiansToRoundedDegrees());
   }


   @Override
   public void bindControls()
   {
      setupControls();

      surfaceNormalFilterParametersProperty.bindBidirectionalBounds(surfaceNormalUpperBoundSlider.valueProperty(),
                                                                    surfaceNormalLowerBoundSlider.valueProperty());
      surfaceNormalFilterParametersProperty.bindBidirectionalUseFilter(enableSurfaceNormalButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.SurfaceNormalFilterParameters, surfaceNormalFilterParametersProperty);
   }

}
