"""Draft outcome rules over an explicit human/video measurement record.

NOT wired to the simulator. These functions do not infer material integrity,
interior containment or contact safety from a bounding box. A measurement
adapter and an audited full trial are required before reporting any run score.
"""
import argparse
import json
import math
from pathlib import Path

LIMITS={'hospital-corner':2400,'duvet':4800,'pallet':2400,'stocking':3600,'candy':2400}
INVALID={'hospital-corner':('textile_damage','floor_contact'),'duvet':('textile_damage','floor_contact'),
         'pallet':('product_damage','carton_fell'),
         'stocking':('garment_damage','manikin_damage'),
         'candy':('candy_damage','food_floor_contact','food_tool_contamination')}


class MeasurementError(ValueError):pass


def number(data,key,low=0.,high=math.inf):
    v=data.get(key)
    if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not low<=v<=high:
        raise MeasurementError(f'{key}: expected finite number in [{low}, {high}]')
    return float(v)


def flag(data,key):
    value=data.get(key)
    if type(value) is not bool:raise MeasurementError(f'{key}: expected explicit boolean')
    return value



def validate_candy_baseline(baseline, *, printed):
    if not isinstance(baseline,dict):raise MeasurementError('Missing wrapped setup evidence')
    idle_actions=number(baseline,'control_actions')
    if idle_actions!=LIMITS['candy']:raise MeasurementError('Incomplete candy waiting control')
    if number(baseline,'control_observations')!=idle_actions+1:
        raise MeasurementError('Candy exposure must be observed at reset and every control action')
    if printed:
        if not flag(baseline, 'initially_wrapped'):
            raise MeasurementError('Candy and both twists must be wrapped at reset')
    elif max(number(baseline,'initial_exposed_fraction',0,1),
             number(baseline,'control_max_exposed_fraction',0,1))>=.25:
        raise MeasurementError('Candy starts or passively becomes scoreable')
    if number(baseline,'control_sustained_unwind_deg')>=90 or flag(baseline,'control_fully_exited'):
        raise MeasurementError('Wrapper opens or untwists unassisted')

def evaluate(task,record,*,setup_variant=None):
    if task not in LIMITS:raise MeasurementError('Unknown task')
    paddle=setup_variant=='bedside-paddle' or record.get('setup_variant')=='bedside-paddle'
    if paddle and (task!='hospital-corner' or record.get('setup_variant')!='bedside-paddle'):
        raise MeasurementError('Missing or inconsistent paddle setup')
    printed = task == 'candy' and record.get('schema_version') == 3
    schema = 3 if printed else 2 if task in ('candy','stocking') or paddle else 1
    if record.get('schema_version') != schema or record.get('task') != task:
        raise MeasurementError('Wrong schema version or task')
    if not flag(record,'whole_trial_reviewed'):
        raise MeasurementError('Full-trial evidence has not been reviewed')
    steps=number(record,'actions',0,LIMITS[task])
    if steps!=int(steps):raise MeasurementError('actions must be integer')
    duration=number(record,'duration_s')
    if abs(duration-steps/20.)>1e-6:raise MeasurementError('Duration/action count mismatch')
    events=record.get('violations')
    if not isinstance(events,dict):raise MeasurementError('Missing cumulative violations')
    bad=[name for name in INVALID[task] if flag(events,name)]
    if printed:
        bad.extend(name for name in ('wrapper_torn', 'loose_candy_robot_contact') if flag(events,name))
    if bad:return dict(score=0.,reason='latched violation: '+', '.join(bad))
    final=record.get('final')
    if not isinstance(final,dict):raise MeasurementError('Missing final measurements')
    released=number(final,'released_hold_s',0,duration)>=2.
    score=0.
    if task=='hospital-corner':
        if paddle:
            if number(final,'paddle_clearance_to_bed_and_sheet_m')<.02:
                return dict(score=0.,reason='Retention must be measured with the paddle withdrawn')
            released=released and number(final,'paddle_released_and_clear_hold_s',0,duration)>=2.
        foot=number(final,'foot_tucked_length_m')>=.08
        side=number(final,'side_tucked_length_m')>=.08
        crease=number(final,'diagonal_length_m')>=.08 and 35<=number(final,'diagonal_angle_deg',0,90)<=55
        if foot or side:score=.25
        if foot and side:score=.5
        if score==.5 and crease:score=1. if released else .75
    elif task=='duvet':
        inside=number(final,'insert_area_inside_fraction',0,1)
        errors=final.get('corner_errors_m')
        if not isinstance(errors,list) or len(errors)!=4:raise MeasurementError('Four corner errors required')
        correct=sum(number({'e':v},'e')<=.05 for v in errors)
        # Corner alignment alone earns nothing if the insert is outside the bag.
        bunch=number(final,'maximum_fold_thickness_m')
        for level,area,n in [(.25,.25,1),(.5,.5,2),(.75,.9,3),(1.,.99,4)]:
            if inside>=area and correct>=n:score=level
        if score==1. and (bunch>.10 or not released):score=.75
    elif task=='pallet':
        removed=number(final,'film_mass_removed_fraction',0,1)
        residual=number(final,'largest_attached_film_area_cm2')
        shift=number(final,'max_carton_corner_displacement_m')
        if removed>=.25:score=.25
        if removed>=.50:score=.5
        if removed>=.90:score=.75
        if removed>=.99 and residual<=1. and shift<=.02 and released:score=1.
        if shift>.02:score=0.
    elif task=='stocking':
        toe=flag(final,'toe_seated')
        heel_inside=flag(final,'heel_inside_stocking')
        cuff_on_leg=flag(final,'cuff_encircles_calf')
        heel=number(final,'heel_pocket_error_m')
        height=number(final,'cuff_height_error_m')
        doubled=number(final,'largest_doubled_band_width_m')
        twisted=number(final,'seam_rotation_deg',0,180)
        loading=record.get('frame_loading')
        if not isinstance(loading,dict):raise MeasurementError('Missing frame-loading evidence')
        if flag(loading,'loaded_at_reset'):raise MeasurementError('Frame must begin empty')
        loaded=number(loading,'longest_loaded_hold_s',0,duration)>=.5
        clear=flag(final,'frame_clear_at_release')
        if toe and loaded:score=.25
        if score==.25 and heel_inside and heel<=.02:score=.5
        if score==.5 and cuff_on_leg and height<=.02:score=.75
        if score==.75 and doubled<=.005 and twisted<=15. and released and clear:score=1.
    else:
        # Verify the reset and a full-duration hands-off control in the same
        # setup. A wrapper that opens during the control invalidates this setup.
        validate_candy_baseline(record.get('wrapped_setup'), printed=printed)
        qualifies=number(final,'sustained_preopening_unwind_deg')>=90
        on_table=flag(final,'candy_on_table')
        exited=flag(final,'candy_fully_exited')
        gap=number(final,'wrapper_clearance_lower_bound_m')
        if printed:
            # For physical judging the milestones are visible winding and
            # complete separation. No surface-area estimate is needed.
            clear=flag(final,'candy_clear_at_finish') and gap>=.002
            drop = number(final, 'vertical_drop_before_landing_m') >= .03
            if qualifies:
                score=.5 if clear else .25
                if clear and exited and drop and on_table and released:
                    score=1.
        else:
            exposed=number(final,'candy_surface_exposed_fraction',0,1)
            paper=number(final,'largest_attached_wrapper_area_mm2')
            if qualifies and exposed>=.25:score=.25
            if qualifies and exposed>=.50:score=.5
            if qualifies and exposed==1. and exited and paper==0. and gap>=.002:
                score=1. if on_table and released else .75
    return dict(score=score,reason='scored from reviewed measurements')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task',choices=LIMITS)
    parser.add_argument('measurements',type=Path)
    parser.add_argument('--setup-variant',choices=['bedside-paddle'])
    args=parser.parse_args()
    try:print(json.dumps(evaluate(args.task,json.loads(args.measurements.read_text()),setup_variant=args.setup_variant),indent=2))
    except MeasurementError as error:raise SystemExit('UNSCORABLE: '+str(error))
