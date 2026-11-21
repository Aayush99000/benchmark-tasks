"""Compare dense native manipulation and waiting trials, then score reviewed evidence."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from outcome_geometry import evaluate, winding_history
from scoring import evaluate as score, LIMITS, INVALID


def load_complete(folder):
    summary = json.loads((folder/'summary.json').read_text())
    rows = [json.loads(line) for line in (folder/'trace.jsonl').read_text().splitlines()]
    if summary['task'] != 'candy' or summary['status'] != 'completed_native_'+summary['motion']:
        raise ValueError(f'{folder}: incomplete candy trial')
    if [r['step'] for r in rows] != list(range(summary['actions']+1)):
        raise ValueError(f'{folder}: incomplete action history')
    if any('wrapper_coverage' not in row for row in rows):
        raise ValueError(f'{folder}: missing an exposure observation')
    return summary, rows


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pair(trial, control, review=None):
    move, motion = load_complete(trial)
    idle, wait = load_complete(control)
    if move['motion'] == 'hold' or idle['motion'] != 'hold' or idle['actions'] != LIMITS['candy']:
        raise ValueError('Expected a manipulation and a full-limit idle control')
    for key in ['variant','geometry','fidelity','control_hz','physics_hz','authored_surface_contacts']:
        if move.get(key) != idle.get(key):
            raise ValueError('Setup mismatch: '+key)
    shared=['native_study.py','geometry.py','candy_measure.py','layout_variants.py','pleated_wrapper.py']
    if move['variant'] in ('faceted-drop','printed-drop'):shared.append('faceted_sweet.py')
    if move['variant']=='printed-drop':shared.append('candy_finish.py')
    for name in shared:
        if move['source_sha256'].get(name) != idle['source_sha256'].get(name):
            raise ValueError('Runtime mismatch: '+name)
    # Native reset includes a physics update, so a GPU contact solve changes the
    # first observed nodes slightly. Check the actual authored inputs exactly.
    assets=['wrapper.usda','wrapper.rest.npy']
    if move['variant'] in ('faceted-drop','printed-drop'):assets.append('candy.usda')
    if move['variant']=='printed-drop':assets.append('wrapper-print.png')
    for name in assets:
        if digest(trial/'assets'/name)!=digest(control/'assets'/name):
            raise ValueError('Different authored material: '+name)
    for name in motion[0]['rigid_objects']:
        np.testing.assert_allclose(motion[0]['rigid_objects'][name],wait[0]['rigid_objects'][name],atol=1e-6,rtol=0)
    reset_delta=np.linalg.norm(np.load(trial/'wrapper-0000.npy')-np.load(control/'wrapper-0000.npy'),axis=1)
    if not np.isfinite(reset_delta).all() or reset_delta.max()>.005:
        raise ValueError(f'Observed material reset differs by {reset_delta.max():.4f} m')
    baseline=dict(initial_exposed_fraction=motion[0]['wrapper_coverage']['exposed_fraction'],
                  control_max_exposed_fraction=max(r['wrapper_coverage']['exposed_fraction'] for r in wait),
                  control_actions=idle['actions'],control_observations=len(wait),
                  control_sustained_unwind_deg=winding_history(wait)['sustained_unwind_deg'],
                  control_fully_exited=any(r['wrapper_coverage']['separation_lower_bound_m']>=.002
                      and r['wrapper_coverage']['exposed_fraction']==1. for r in wait))
    outcome=evaluate(trial)
    held=0
    for r in reversed(motion):
        center=np.array(r['rigid_objects']['candy'][:3])
        lower,upper=np.asarray(r['surfaces']['wrapper']['bounds_m'])
        def cleared(jaw):
            point=np.asarray(jaw['tcp_m'])
            wrapper_gap=np.linalg.norm(np.maximum(np.maximum(lower-point,point-upper),0.))
            return (-sum(jaw['finger_joints'])>=.075 and np.linalg.norm(center-point)>=.080
                    and wrapper_gap>=.080)
        if not all(cleared(jaw) for jaw in r['grippers'].values()):break
        held+=1
    printed = move['variant'] == 'printed-drop'
    final=dict(candy_surface_exposed_fraction=outcome['final_exposed_fraction'],
               sustained_preopening_unwind_deg=outcome['grasped_sustained_unwind_deg'],
               candy_on_table=outcome['core_on_table_through_last_two_seconds'],
               candy_fully_exited=outcome['full_exit_final_hold'],
               wrapper_clearance_lower_bound_m=outcome['final_wrapper_clearance_lower_bound_m'],
               released_hold_s=max(0,held-1)/20.)
    if printed:
        baseline['initially_wrapped'] = (
            baseline['initial_exposed_fraction'] < .25
            and baseline['control_max_exposed_fraction'] < .25
            and all(abs(value) >= 180. for value in outcome['pregrasp_tail_winding_deg'])
        )
        final['candy_clear_at_finish'] = (outcome['final_exposed_fraction'] == 1.
                                          and final['wrapper_clearance_lower_bound_m'] >= .002)
        escape = next((index for index, row in enumerate(motion)
                       if row['wrapper_coverage']['exposed_fraction'] == 1.
                       and row['wrapper_coverage']['separation_lower_bound_m'] >= .002), None)
        # The detail camera shows the fall. For the trace, measure the
        # descent just before the first observed full separation.
        drop = 0.
        if escape is not None:
            height = motion[escape]['rigid_objects']['candy'][2]
            previous = motion[max(0, escape-10):escape+1]
            drop = max(row['rigid_objects']['candy'][2] - height for row in previous)
        final['vertical_drop_before_landing_m'] = drop
    # Intersection-free disjoint envelopes certify no attached paper. If that
    # proof is absent, measuring a residual area requires human observation.
    if final['candy_fully_exited']:final['largest_attached_wrapper_area_mm2']=0.
    result=dict(trial=trial.name,control=control.name,wrapped_setup=baseline,
                maximum_observed_reset_difference_m=float(reset_delta.max()),
                final_geometry=outcome,final_measurements=final,task_score=None,
                trial_trace_sha256=digest(trial/'trace.jsonl'),control_trace_sha256=digest(control/'trace.jsonl'))
    if review is not None:
        if review['trial_trace_sha256']!=result['trial_trace_sha256'] or review['whole_trial_reviewed'] is not True:
            raise ValueError('Review must cover this complete trial')
        if not printed and 'largest_attached_wrapper_area_mm2' not in final:
            final['largest_attached_wrapper_area_mm2']=review['largest_attached_wrapper_area_mm2']
        violation_names = INVALID['candy']
        if printed:
            violation_names += ('wrapper_torn', 'loose_candy_robot_contact')
        rec=dict(task='candy',schema_version=3 if printed else 2,actions=move['actions'],duration_s=move['actions']/20.,
                 whole_trial_reviewed=True,wrapped_setup=baseline,final=final,
                 violations={key:review['violations'][key] for key in violation_names})
        result.update(task_score=score('candy',rec)['score'],scorer_input=rec)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial',type=Path)
    parser.add_argument('control',type=Path)
    parser.add_argument('--review',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=pair(args.trial,args.control,json.loads(args.review.read_text()) if args.review else None)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))
