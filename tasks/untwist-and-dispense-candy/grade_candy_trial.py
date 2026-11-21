"""Score a completed native faceted-sweet trial under the rigid-sweet simulation model."""
import argparse
import json
from pathlib import Path

from candy_contact_audit import evidence
from candy_observations import digest, load_complete, pair
from scoring import validate_candy_baseline


def grade(trial,control,*,physics_only=False):
    summary, rows = load_complete(trial)
    printed = summary['variant'] == 'printed-drop'
    escape = None
    if printed:
        escape = next((row['step'] for row in rows
                       if row['wrapper_coverage']['exposed_fraction'] == 1.
                       and row['wrapper_coverage']['separation_lower_bound_m'] >= .002), None)
    contacts=evidence(trial, loose_from_step=escape)
    observed=pair(trial,control)
    if physics_only:
        if summary['recording'] or summary['frames_per_camera']:
            raise ValueError('Use the recorded grading path for a camera run')
    elif not summary['recording'] or summary['frames_per_camera']!=summary['actions']+1:
        raise ValueError('Full native camera recording required')
    if set(summary['final_state']['rigid_objects'])!={'candy'}:
        raise ValueError('Unexpected rigid objects for this candy setup')
    if printed and observed['final_measurements']['sustained_preopening_unwind_deg'] < 90.:
        # Every tier requires this milestone. An uncertain contact cannot
        # change a zero, so preserve the measured clearance bounds as-is.
        validate_candy_baseline(observed['wrapped_setup'], printed=True)
        out = observed
        out.update(task_score=0., zero_reason='No held 90-degree untwist before exit.')
    else:
        if printed and escape is not None and contacts['minimum_loose_candy_tip_clearance_m'] <= .001:
            raise ValueError('Tip clearance from loose candy is inconclusive. Review possible contact.')

        if contacts['minimum_unclean_surface_clearance_m']<=0:
            raise ValueError('A clean-contact certificate is unavailable. Inspect the possible contact before scoring.')
        if contacts['minimum_floor_clearance_m']<=.001:
            raise ValueError('Possible floor contact. Inspect the floor segment before scoring.')
        # The rigid benchmark represents the sweet as one immutable body. This
        # flag applies only to that model; it is not a real fracture measurement.
        review=dict(trial_trace_sha256=digest(trial/'trace.jsonl'),whole_trial_reviewed=True,
                    violations=dict(candy_damage=False,food_floor_contact=False,food_tool_contamination=False))
        if printed:
            # One invariant surface mesh models the continuous uncut sheet.
            review['violations'].update(wrapper_torn=False, loose_candy_robot_contact=False)
        elif 'largest_attached_wrapper_area_mm2' not in observed['final_measurements']:
            raise ValueError('Remaining paper area requires observation before grading')
        out=pair(trial,control,review)
    out.update(scope='Native simulation with one rigid, nonfracturing hard sweet. Physical fracture, sheet tear and hygiene require real-world validation.',
               full_task_demonstrated=out['task_score']==1.,contact_audit=contacts,
               camera_sha256={name:digest(trial/(name+'.mp4')) for name in summary['video_stats']})
    if physics_only:
        out.update(full_task_demonstrated=False,physics_only_repeat=True)
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('trial',type=Path)
    p.add_argument('control',type=Path)
    p.add_argument('--physics-only',action='store_true',help='Grade a state-only replication separately from a recorded demonstration.')
    args=p.parse_args()
    out=grade(args.trial,args.control,physics_only=args.physics_only)
    (args.trial/('physics-grade.json' if args.physics_only else 'grade.json')).write_text(json.dumps(out,indent=2)+'\n')
    print(out['task_score'],out['final_measurements'],out['contact_audit']['minimum_unclean_surface_clearance_m'])
