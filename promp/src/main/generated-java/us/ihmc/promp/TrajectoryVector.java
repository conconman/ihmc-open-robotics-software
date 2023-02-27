// Targeted by JavaCPP version 1.5.8: DO NOT EDIT THIS FILE

package us.ihmc.promp;

import java.nio.*;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import static us.ihmc.promp.global.promp.*;

@Name("std::vector<promp::Trajectory>") @Properties(inherit = us.ihmc.promp.presets.ProMPInfoMapper.class)
public class TrajectoryVector extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public TrajectoryVector(Pointer p) { super(p); }
    public TrajectoryVector(Trajectory value) { this(1); put(0, value); }
    public TrajectoryVector(Trajectory ... array) { this(array.length); put(array); }
    public TrajectoryVector()       { allocate();  }
    public TrajectoryVector(long n) { allocate(n); }
    private native void allocate();
    private native void allocate(@Cast("size_t") long n);
    public native @Name("operator =") @ByRef TrajectoryVector put(@ByRef TrajectoryVector x);

    public boolean empty() { return size() == 0; }
    public native long size();
    public void clear() { resize(0); }
    public native void resize(@Cast("size_t") long n);

    @Index(function = "at") public native @ByRef Trajectory get(@Cast("size_t") long i);
    public native TrajectoryVector put(@Cast("size_t") long i, Trajectory value);

    public native @ByVal Iterator insert(@ByVal Iterator pos, @ByRef Trajectory value);
    public native @ByVal Iterator erase(@ByVal Iterator pos);
    public native @ByVal Iterator begin();
    public native @ByVal Iterator end();
    @NoOffset @Name("iterator") public static class Iterator extends Pointer {
        public Iterator(Pointer p) { super(p); }
        public Iterator() { }

        public native @Name("operator ++") @ByRef Iterator increment();
        public native @Name("operator ==") boolean equals(@ByRef Iterator it);
        public native @Name("operator *") @ByRef @Const Trajectory get();
    }

    public Trajectory[] get() {
        Trajectory[] array = new Trajectory[size() < Integer.MAX_VALUE ? (int)size() : Integer.MAX_VALUE];
        for (int i = 0; i < array.length; i++) {
            array[i] = get(i);
        }
        return array;
    }
    @Override public String toString() {
        return java.util.Arrays.toString(get());
    }

    public Trajectory pop_back() {
        long size = size();
        Trajectory value = get(size - 1);
        resize(size - 1);
        return value;
    }
    public TrajectoryVector push_back(Trajectory value) {
        long size = size();
        resize(size + 1);
        return put(size, value);
    }
    public TrajectoryVector put(Trajectory value) {
        if (size() != 1) { resize(1); }
        return put(0, value);
    }
    public TrajectoryVector put(Trajectory ... array) {
        if (size() != array.length) { resize(array.length); }
        for (int i = 0; i < array.length; i++) {
            put(i, array[i]);
        }
        return this;
    }
}

