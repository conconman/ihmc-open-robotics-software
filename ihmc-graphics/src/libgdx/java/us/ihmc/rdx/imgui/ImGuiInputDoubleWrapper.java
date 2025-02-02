package us.ihmc.rdx.imgui;

import imgui.type.ImDouble;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ImGuiInputDoubleWrapper extends ImGuiFancyWidget
{
   private final ImDoubleWrapper imDoubleWrapper;
   private final Runnable onUserModified;

   public ImGuiInputDoubleWrapper(String label,
                                  String format,
                                  double step,
                                  double stepFast,
                                  DoubleSupplier wrappedValueGetter,
                                  DoubleConsumer wrappedValueSetter,
                                  Runnable onUserModified)
   {
      super(label, format);
      this.onUserModified = onUserModified;
      imDoubleWrapper = new ImDoubleWrapper(wrappedValueGetter, wrappedValueSetter, imDouble -> render(step, stepFast, imDouble));
   }

   private void render(double step, double stepFast, ImDouble imDouble)
   {
      beforeWidgetRender();
      if (ImGuiTools.volatileInputDouble(label, imDouble, step, stepFast, format))
         onUserModified.run();
      afterWidgetRender();
   }

   public void render()
   {
      imDoubleWrapper.renderImGuiWidget();
   }
}
