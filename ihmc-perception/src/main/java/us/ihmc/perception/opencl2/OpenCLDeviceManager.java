package us.ihmc.perception.opencl2;

import org.bytedeco.opencl._cl_context;

public interface OpenCLDeviceManager
{
   _cl_context createContext();
}
