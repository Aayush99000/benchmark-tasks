"""Audit recorded geometry and measured cutter travel, without running a controller."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scoring import coverage, tucks
from film import EXIT, WIDTH

ACTION_LIMIT=3600
REQUIRED_SOURCES={
    'arena_task.py','assets.py','build_contact_kernel.py','clearance.py',
    'contact_fast.py','contact_kernel.cpp','contact_kernel_build.json','film.py',
    'launch.py','prepare_runtime.py','run.py','scoring.py','station_setup.py',
    'tuck_measurement.py'}


def audit_sources(run,source_root=None):
    """Check run.py's source_* snapshots, manifest and audited runtime revision.

    The verifier and tests need not match the current auditor, but every stored
    snapshot must still match its recorded startup hash. This establishes source
    correspondence; it is not cryptographic proof of executed code.
    """
    run=Path(run);root=Path(source_root) if source_root else Path(__file__).resolve().parent
    failures=[];matched=[];checked=[]
    try:hashes=json.loads((run/'manifest.json').read_text())['sources']
    except (OSError,ValueError,KeyError,TypeError) as exc:
        return dict(verified=False,failures=['Missing or invalid source manifest: '+str(exc)],matched_sources=[])
    if not isinstance(hashes,dict):
        return dict(verified=False,failures=['Source hash manifest must be an object'],matched_sources=[])
    for name in sorted(REQUIRED_SOURCES-set(hashes)):
        failures.append('Required runtime source missing from manifest: '+name)
    for name,digest in sorted(hashes.items()):
        if (not isinstance(name,str) or Path(name).name!=name or '/' in name or '\\' in name
            or not isinstance(digest,str) or len(digest)!=64
            or any(c not in '0123456789abcdef' for c in digest)):
            failures.append('Invalid source manifest entry: '+str(name));continue
        snapshot=run/('source_'+name)
        if not snapshot.is_file() or hashlib.sha256(snapshot.read_bytes()).hexdigest()!=digest:
            failures.append('Startup source snapshot missing or hash mismatch: '+name)
        else:checked.append(name)
        if name in REQUIRED_SOURCES:
            current=root/name
            if not current.is_file() or hashlib.sha256(current.read_bytes()).hexdigest()!=digest:
                failures.append('Run differs from audited runtime source: '+name)
            else:matched.append(name)
    return dict(verified=not failures,failures=failures,matched_sources=matched,
                snapshot_files_checked=len(checked),audited_source_root=str(root.resolve()))


def audit_episode_contract(trace,final,total_steps):
    steps=[s.get('step') for s in trace]+[final.get('step'),total_steps]
    failures=[]
    if any(type(s) is not int or not 0<=s<=ACTION_LIMIT for s in steps):
        failures.append('Episode exceeds 3600 actions or has invalid action counts')
    if final.get('step')!=total_steps:
        failures.append('Controller and final scorer action counts disagree')
    return failures


def verify(run):
    source_audit=audit_sources(run)
    failures = list(source_audit['failures'])
    def need(condition, message):
        if not condition:
            failures.append(message)

    trace = json.loads((run / 'score_trace.json').read_text())
    result = json.loads((run / 'result.json').read_text())
    final = result['score']
    failures.extend(audit_episode_contract(trace,final,result.get('steps')))
    snapshots = sorted(run.glob('state_*.npz'))
    need(bool(trace) and len(snapshots) > 1, 'Missing continuous evidence')
    need(final.get('scoring_version') == '2.0', 'Run uses historical scoring; a fresh v2 run is required')
    need(all(b['step'] - a['step'] == 5 for a, b in zip(trace, trace[1:])),
         'Gaps in measured score trace')
    need(not trace[0]['both_edges_grasped'] and not trace[0]['cut'],
         'Film already manipulated at reset')
    need(all(not s['invalid'] for s in trace), 'Invalid material or fruit state')
    grasp = next((s['step'] for s in trace if s['both_edges_grasped']), None)
    tucked = next((s['step'] for s in trace if all(s['underside_tucks'])), None)
    cut = next((s['step'] for s in trace if s['cut']), None)
    need(grasp is not None and tucked is not None and cut is not None
         and grasp < tucked < cut, 'Missing measured grasp, tuck and cut order')
    need(final['score'] == 1. and final['full_task_success'], 'Final score below 1.0')
    need(final['cut_fraction'] == 1. and final['active_grasps'] == 0,
         'Incomplete cut or film still grasped')
    need(final['robot_clearance_m'] >= .020 and final['stable_seconds'] >= 1.,
         'Wrap not retained with hands clear for one second')
    need(.30 <= final['dispensed_m'] <= .65, 'Unsuitable film payout')
    cutter = []
    last = None
    for path in snapshots:
        with np.load(path) as state:
            need(np.isfinite(state['film']).all(), 'Nonfinite film geometry')
            cutter.append(state['cutter'].copy())
            last = {key: state[key].copy() for key in state.files}
    cutter = np.asarray(cutter)
    aligned = ((abs(cutter[:, 0] - EXIT[0]) < .008)
               & (abs(cutter[:, 2] - EXIT[2] - .023) < .010))
    sweep = cutter[aligned, 1]
    need(len(sweep) > 1 and sweep.max() >= WIDTH / 2 - .005
         and sweep.min() <= -WIDTH / 2 + .005,
         'No measured full-width native cutter sweep')
    measured_coverage = coverage(last['film'], last['faces'],
                                 last['melon_pos'], last['melon_rot'])
    measured_tucks = tucks(last['film'], last['melon_pos'], last['melon_rot'], last['faces'])
    need(measured_coverage >= .95 and all(measured_tucks),
         'Final geometry lacks face coverage or both underside tucks')
    need((last['aperture'] > .055).all(), 'Final native jaws are not open')
    return dict(full_score_verified=not failures, failures=failures,
                source_audit=source_audit,action_limit=ACTION_LIMIT,
                score=final['score'], steps=final['step'],
                measured_coverage=measured_coverage, measured_tucks=measured_tucks,
                cutter_y_range_m=[float(sweep.min()), float(sweep.max())]
                if len(sweep) else None,
                events={'both_edges_seen': grasp, 'both_tucks_seen': tucked,
                        'cut_completed': cut},
                scope='Measured hybrid simulation only. Film uses reduced PBD '
                      'and measured-pinch surrogates; no hardware calibration.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    result = verify(args.run)
    (args.run / 'verification.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['full_score_verified'] else 1)
