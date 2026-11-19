"""Optional CPU accelerator; builds only the included original C++ source."""
from pathlib import Path
import subprocess,hashlib,json
p=Path(__file__).resolve().parent
subprocess.run(['c++','-O3','-std=c++17','-shared','-fPIC',str(p/'contact_kernel.cpp'),'-o',str(p/'contact_kernel.so')],check=True)
(p/'contact_kernel_build.json').write_text(json.dumps({'source_sha256':hashlib.sha256((p/'contact_kernel.cpp').read_bytes()).hexdigest(),'binary_sha256':hashlib.sha256((p/'contact_kernel.so').read_bytes()).hexdigest(),'flags':['-O3','-std=c++17','-shared','-fPIC'],'fast_math':False},indent=2))
print('Built contact_kernel.so')
