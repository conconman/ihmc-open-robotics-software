// Targeted by JavaCPP version 1.5.8: DO NOT EDIT THIS FILE

package us.ihmc.perception.slamWrapper;

import java.nio.*;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

public class SlamWrapper extends us.ihmc.perception.slamWrapper.presets.SlamWrapperInfoMapper {
    static { Loader.load(); }

// Parsed from include/FactorGraphExternal.h

// #pragma once

// #include "FactorGraphHandler.h"

public static class FactorGraphExternal extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public FactorGraphExternal() { super((Pointer)null); allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(long)}. */
    public FactorGraphExternal(long size) { super((Pointer)null); allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public FactorGraphExternal(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(long size);
    @Override public FactorGraphExternal position(long position) {
        return (FactorGraphExternal)super.position(position);
    }
    @Override public FactorGraphExternal getPointer(long i) {
        return new FactorGraphExternal((Pointer)this).offsetAddress(i);
    }

      // Expects packed Pose3 as XYZYPR
      public native void addPriorPoseFactor(int index, FloatPointer pose);
      public native void addPriorPoseFactor(int index, FloatBuffer pose);
      public native void addPriorPoseFactor(int index, float[] pose);

      // Expects packed Pose3 as XYZYPR
      public native void addOdometryFactor(int poseId, FloatPointer odometry);
      public native void addOdometryFactor(int poseId, FloatBuffer odometry);
      public native void addOdometryFactor(int poseId, float[] odometry);

      // Expects packed Vector4
      public native void addOrientedPlaneFactor(int lmId, int poseIndex, FloatPointer lmMean);
      public native void addOrientedPlaneFactor(int lmId, int poseIndex, FloatBuffer lmMean);
      public native void addOrientedPlaneFactor(int lmId, int poseIndex, float[] lmMean);

      public native void optimize();

      public native void optimizeISAM2(@Cast("uint8_t") byte numberOfUpdates);

      public native void clearISAM2();

      // Expects packed Pose3
      public native void setPoseInitialValue(int index, FloatPointer value);
      public native void setPoseInitialValue(int index, FloatBuffer value);
      public native void setPoseInitialValue(int index, float[] value);

      // Expects packed OrientedPlane3
      public native void setOrientedPlaneInitialValue(int landmarkId, FloatPointer value);
      public native void setOrientedPlaneInitialValue(int landmarkId, FloatBuffer value);
      public native void setOrientedPlaneInitialValue(int landmarkId, float[] value);

      // Expects packed Vector6
      public native void createOdometryNoiseModel(FloatPointer odomVariance);
      public native void createOdometryNoiseModel(FloatBuffer odomVariance);
      public native void createOdometryNoiseModel(float[] odomVariance);

      // Expects packed Vector3
      public native void createOrientedPlaneNoiseModel(FloatPointer lmVariances);
      public native void createOrientedPlaneNoiseModel(FloatBuffer lmVariances);
      public native void createOrientedPlaneNoiseModel(float[] lmVariances);

      // Expects 4x4 homogenous transform matrix as 16-double array
      public native void addOdometryFactorSE3(int poseId, DoublePointer odometry);
      public native void addOdometryFactorSE3(int poseId, DoubleBuffer odometry);
      public native void addOdometryFactorSE3(int poseId, double[] odometry);

      // Expects 4x4 homogenous transform as 16-double array
      public native void setPoseInitialValueSE3(int index, DoublePointer value);
      public native void setPoseInitialValueSE3(int index, DoubleBuffer value);
      public native void setPoseInitialValueSE3(int index, double[] value);

      // Add Prior Pose Factor with full 4x4 homogenous SE3 matrix
      public native void addPriorPoseFactorSE3(int poseId, DoublePointer pose);
      public native void addPriorPoseFactorSE3(int poseId, DoubleBuffer pose);
      public native void addPriorPoseFactorSE3(int poseId, double[] pose);

      public native @Cast("bool") boolean getPoseById(int poseId, DoublePointer pose);
      public native @Cast("bool") boolean getPoseById(int poseId, DoubleBuffer pose);
      public native @Cast("bool") boolean getPoseById(int poseId, double[] pose);

      public native @Cast("bool") boolean getPlanarLandmarkById(int poseId, DoublePointer plane);
      public native @Cast("bool") boolean getPlanarLandmarkById(int poseId, DoubleBuffer plane);
      public native @Cast("bool") boolean getPlanarLandmarkById(int poseId, double[] plane);

      public native void printResults();

      public native void helloWorldTest();
}

}
