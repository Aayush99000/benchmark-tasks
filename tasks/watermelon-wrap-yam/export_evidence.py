"""Package selected simulator evidence without credentials or unrelated workspace files."""
from pathlib import Path
import argparse,zipfile,json,base64,hashlib,shutil
p=argparse.ArgumentParser();p.add_argument('trial',type=Path);a=p.parse_args();d=a.trial
files=[]
for name in ['collider_check.json','controller_trace.json','command_history.jsonl','command_result.json','live_result.json','score_trace.json','manifest.json','result.json','error.txt','development_run.mp4']:
 if (d/name).exists():files.append(d/name)
files+=list(d.glob('source_*'))
states=sorted(d.glob('state_*.npz'))
if states:files+=states[-2:]
with zipfile.ZipFile(d/'evidence.zip','w',zipfile.ZIP_DEFLATED) as z:
 for f in files:z.write(f,f.name)
data=(d/'evidence.zip').read_bytes();encoded=base64.b64encode(data).decode()
(d/'evidence-transfer.txt').write_text(encoded)
(d/'evidence-manifest.json').write_text(json.dumps({'bytes':len(data),'base64_chars':len(encoded),'sha256':hashlib.sha256(data).hexdigest(),'files':[p.name for p in files]},indent=2))
if (d/'frames').exists():
 frames=sorted((d/'frames').glob('*.png'))
 if frames:shutil.copy2(frames[-1],d/'latest.png')
print((d/'evidence-manifest.json').read_text())
