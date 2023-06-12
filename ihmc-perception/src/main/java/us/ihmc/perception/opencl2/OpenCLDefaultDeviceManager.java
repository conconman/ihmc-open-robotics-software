package us.ihmc.perception.opencl2;

import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl._cl_context;
import org.bytedeco.opencl._cl_device_id;
import org.bytedeco.opencl._cl_platform_id;

import static org.bytedeco.opencl.global.OpenCL.*;

/**
 * Gets access to the default OpenCL device and allows you to create contexts
 */
public class OpenCLDefaultDeviceManager implements OpenCLDeviceManager
{
   private final _cl_platform_id platformId = new _cl_platform_id();
   private final _cl_device_id deviceId = new _cl_device_id();

   public OpenCLDefaultDeviceManager()
   {
      IntPointer platformCount = new IntPointer(1);
      clGetPlatformIDs(1, platformId, platformCount);

      IntPointer deviceCount = new IntPointer(1);
      clGetDeviceIDs(platformId, CL_DEVICE_TYPE_DEFAULT, 1, deviceId, deviceCount);
   }

   @Override
   public _cl_context createContext()
   {
      IntPointer returnCode = new IntPointer(1);
      return clCreateContext(null, 1, deviceId, null, null, returnCode);
   }
}
