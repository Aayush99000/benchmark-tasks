"""Optional exact point/OBB accelerator. Falls back when it is not built."""
import ctypes,os
from pathlib import Path
import numpy as np
kernel=None
path=Path(__file__).with_name('contact_kernel.so')
if path.exists() and os.environ.get('MELON_PYTHON_CONTACTS')!='1':
    lib=ctypes.CDLL(str(path));kernel=lib.project_boxes
    array=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
    mask=np.ctypeslib.ndpointer(dtype=np.uint8,flags='C_CONTIGUOUS')
    kernel.argtypes=[array,ctypes.c_long,array,array,array,ctypes.c_long,mask];kernel.restype=None

def project(flat,transforms,bounds,aabbs,pinned):
    kernel(flat,len(flat),transforms,bounds,aabbs,len(bounds),pinned)
