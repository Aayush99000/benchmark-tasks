"""Make a disposable exact-source copy without altering the submitted task.

Only the three original publication metadata snapshots may differ from the
submitted task. Every runtime/plan file must already match the native freeze.
"""
from pathlib import Path
import argparse
import hashlib
import json
import shutil


def prepare(canonical, output):
    here = Path(__file__).resolve().parent
    canonical, output = canonical.resolve(), output.resolve()
    if output == canonical or canonical in output.parents:
        raise ValueError('Reproduction output must be outside the submitted task')
    expected = json.loads((here / 'source-freeze.json').read_text())['all_file_sha256']
    metadata = {'README.md', 'task.json', 'MATERIALS.md'}
    selected = {}
    for name, digest in expected.items():
        source = here / 'frozen-metadata' / name if name in metadata else canonical / name
        if hashlib.sha256(source.read_bytes()).hexdigest() != digest:
            raise ValueError('Frozen reproduction input differs: ' + name)
        selected[name] = source
    output.mkdir(parents=True, exist_ok=False)
    for name, source in selected.items():
        shutil.copyfile(source, output / name)
    actual = {name: hashlib.sha256((output / name).read_bytes()).hexdigest() for name in expected}
    if actual != expected:
        raise RuntimeError('Reproduction copy hash mismatch')
    return dict(status='EXACT_FROZEN_COPY_READY', files=len(actual), source_unchanged=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--canonical', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(prepare(args.canonical, args.out)))
