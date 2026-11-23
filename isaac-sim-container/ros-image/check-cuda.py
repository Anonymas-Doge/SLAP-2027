#!/usr/bin/env python3
"""Exercise CUDA runtime allocation and memory operations on the selected device."""
import ctypes as c

cuda = c.CDLL("libcudart.so.13")
cuda.cudaGetErrorString.restype = c.c_char_p

def check(code):
    if code:
        raise RuntimeError(cuda.cudaGetErrorString(code).decode())

ptr = c.c_void_p()
size = c.c_size_t(4096)
check(cuda.cudaMalloc(c.byref(ptr), size))
try:
    check(cuda.cudaMemset(ptr, 0, size))
    check(cuda.cudaDeviceSynchronize())
finally:
    check(cuda.cudaFree(ptr))
print("CUDA allocation and memset passed.")
