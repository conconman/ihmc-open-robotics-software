// Targeted by JavaCPP version 1.5.8: DO NOT EDIT THIS FILE

package us.ihmc.perception.zedDriver;

import java.nio.*;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

public class ZEDOpenDriver extends us.ihmc.perception.zedDriver.presets.ZEDDriverInfoMapper {
    static { Loader.load(); }

// Parsed from include/zed_open_driver_external.h

// #include "zed_open_driver.h"

@NoOffset public static class ZEDOpenDriverExternal extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public ZEDOpenDriverExternal(Pointer p) { super(p); }

      public ZEDOpenDriverExternal(int resolution, int fps) { super((Pointer)null); allocate(resolution, fps); }
      private native void allocate(int resolution, int fps);

      public native @Cast("bool") boolean getFrameStereoYUVExternal(@Cast("uint8_t*") BytePointer yuvBytes, IntPointer dims);
      public native @Cast("bool") boolean getFrameStereoYUVExternal(@Cast("uint8_t*") ByteBuffer yuvBytes, IntBuffer dims);
      public native @Cast("bool") boolean getFrameStereoYUVExternal(@Cast("uint8_t*") byte[] yuvBytes, int[] dims);

      public native @Cast("bool") boolean getFrameDimensions(IntPointer dims);
      public native @Cast("bool") boolean getFrameDimensions(IntBuffer dims);
      public native @Cast("bool") boolean getFrameDimensions(int[] dims);
        

}

}
