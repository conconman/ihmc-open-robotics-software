// Targeted by JavaCPP version 1.5.7: DO NOT EDIT THIS FILE

package us.ihmc.bytedeco.mapsenseWrapper;

import java.nio.*;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

public class VisualOdometry extends us.ihmc.bytedeco.mapsenseWrapper.presets.VisualOdometryInfoMapper {
    static { Loader.load(); }

// Parsed from include/visual_odometry_external.h

// #pragma once

// #include "core.h"
// #include "application_state.h"
// #include "visual_odometry.h"

@NoOffset public static class VisualOdometryExternal extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public VisualOdometryExternal(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(long)}. */
    public VisualOdometryExternal(long size) { super((Pointer)null); allocateArray(size); }
    private native void allocateArray(long size);
    @Override public VisualOdometryExternal position(long position) {
        return (VisualOdometryExternal)super.position(position);
    }
    @Override public VisualOdometryExternal getPointer(long i) {
        return new VisualOdometryExternal((Pointer)this).offsetAddress(i);
    }

        public VisualOdometryExternal() { super((Pointer)null); allocate(); }
        private native void allocate();

        public native void printMat(FloatPointer buffer, int height, int width);
        public native void printMat(FloatBuffer buffer, int height, int width);
        public native void printMat(float[] buffer, int height, int width);
        public native void printMat(@Cast("uint8_t*") BytePointer buffer, int height, int width);
        public native void printMat(@Cast("uint8_t*") ByteBuffer buffer, int height, int width);
        public native void printMat(@Cast("uint8_t*") byte[] buffer, int height, int width);
        
        public native void displayMat(@Cast("uint8_t*") BytePointer buffer, int height, int width, int delayMilliSeconds);
        public native void displayMat(@Cast("uint8_t*") ByteBuffer buffer, int height, int width, int delayMilliSeconds);
        public native void displayMat(@Cast("uint8_t*") byte[] buffer, int height, int width, int delayMilliSeconds);
        
        public native void updateMonocular(@Cast("uint8_t*") BytePointer buffer, int height, int width);
        public native void updateMonocular(@Cast("uint8_t*") ByteBuffer buffer, int height, int width);
        public native void updateMonocular(@Cast("uint8_t*") byte[] buffer, int height, int width);
        public native void updateStereo(@Cast("uint8_t*") BytePointer bufferLeft, @Cast("uint8_t*") BytePointer bufferRight, int height, int width);
        public native void updateStereo(@Cast("uint8_t*") ByteBuffer bufferLeft, @Cast("uint8_t*") ByteBuffer bufferRight, int height, int width);
        public native void updateStereo(@Cast("uint8_t*") byte[] bufferLeft, @Cast("uint8_t*") byte[] bufferRight, int height, int width);
}

}
