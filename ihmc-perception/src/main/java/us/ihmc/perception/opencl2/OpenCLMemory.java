package us.ihmc.perception.opencl2;

import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencl._cl_context;
import org.bytedeco.opencl._cl_mem;

import java.io.Closeable;
import java.io.IOException;

import static org.bytedeco.opencl.global.OpenCL.*;

public class OpenCLMemory implements Closeable
{
   private final int bytes;
   private final long length;
   private final _cl_mem handle;

   public OpenCLMemory(_cl_context context, int bytes, long length, Pointer hostPointer)
   {
      this.bytes = bytes;
      this.length = length;

      int flags = CL_MEM_READ_WRITE;

      if (hostPointer != null)
         flags |= CL_MEM_USE_HOST_PTR;

      IntPointer returnCode = new IntPointer(1);

      handle = clCreateBuffer(context, flags, bytes * length, hostPointer, returnCode);
   }

   public int getBytes()
   {
      return bytes;
   }

   public long getLength()
   {
      return length;
   }

   public _cl_mem getHandle()
   {
      if (handle.isNull())
         throw new RuntimeException("Tried to access de-referenced OpenCL memory");
      return handle;
   }

   @Override
   public void close() throws IOException
   {
      clReleaseMemObject(handle);
      handle.close();
   }
}
