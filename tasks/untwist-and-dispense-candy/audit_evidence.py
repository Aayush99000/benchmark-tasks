"""Audit native study completion, video encoding and last-second settling.

This cannot validate the material model or award a task score. A solver
returning successfully is not treated as a successful physical task.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def audit(folder):
    summary=json.loads((folder/'summary.json').read_text())
    result=dict(task=summary['task'],native_status=summary['status'],
                full_task_demonstrated=summary['full_task_demonstrated'],
                task_score=summary['task_score'],source_unchanged=summary.get('source_unchanged'),
                videos={},settling={},evidence=str(folder))
    runtime=('native_study.py','geometry.py','review_views.py')
    source=folder/'source'
    if 'motion' in summary:
        runtime = tuple(summary['source_sha256'])
    result['frozen_runtime_matches_recorded_hashes']=all(
        (source/name).exists() and hashlib.sha256((source/name).read_bytes()).hexdigest()==summary['source_sha256'][name]
        for name in runtime)
    # The study tracks every Python file in the package, including unrelated
    # draft/scorer utilities. Report the runtime subset separately, never erase
    # the original broad source_unchanged result.
    here=Path(__file__).resolve().parent
    result['current_runtime_matches_recorded_hashes']=all(
        (here/name).exists() and hashlib.sha256((here/name).read_bytes()).hexdigest()==summary['source_sha256'][name]
        for name in runtime)
    if summary['status']!='completed_native_'+summary.get('motion','hold'):
        result.update(status='failed_to_complete',error=summary.get('error'));return result
    rows=[json.loads(line) for line in (folder/'trace.jsonl').read_text().splitlines()]
    if len(rows)!=summary['actions']+1 or any(r['step']!=i for i,r in enumerate(rows)):
        raise ValueError('Incomplete trace: '+str(folder))
    end=rows[-1]['time_s'];tail=[r for r in rows if r['time_s']>=end-1.]
    # For moving trials quantify the fixed, pre-approach window used by the
    # actual playback. The final second contains commanded robot motion.
    if summary.get('motion','hold') != 'hold':
        tail=[r for r in rows if 4.<=r['time_s']<=5.]
        result['settling_window_s']=[4.,5.]
    for name in rows[-1]['surfaces']:
        samples=[r['surfaces'][name] for r in tail]
        if any('max_vertex_speed_m_s' not in s for s in samples):
            result['settling'][name]=dict(status='unmeasured');continue
        speed=max(s['max_vertex_speed_m_s'] for s in samples)
        bounds=[s['bounds_m'][0]+s['bounds_m'][1] for s in samples]
        drift=max(max(b[i] for b in bounds)-min(b[i] for b in bounds) for i in range(6))
        result['settling'][name]=dict(window_max_speed_m_s=speed,
            window_max_aabb_edge_change_m=drift,
            candidate_settled=speed<=.03 and drift<=.005)
    for path in sorted(folder.glob('*.mp4')):
        r=subprocess.run(['ffprobe','-v','error','-count_frames','-select_streams','v:0',
            '-show_entries','stream=codec_name,pix_fmt,avg_frame_rate,nb_read_frames,width,height',
            '-of','json',str(path)],capture_output=True,text=True,check=True)
        stream=json.loads(r.stdout)['streams'][0]
        valid=(stream['codec_name']=='h264' and stream['pix_fmt']=='yuv420p'
               and stream['avg_frame_rate']=='20/1' and int(stream['nb_read_frames'])==len(rows))
        result['videos'][path.name]=dict(stream,valid=valid,size_bytes=path.stat().st_size)
    result['status']='native_study_audited'
    result['all_videos_valid']=(len(result['videos'])==4 and all(v['valid'] for v in result['videos'].values())
                                if summary.get('recording',True) else None)
    result['all_materials_candidate_settled']=all(v.get('candidate_settled',False) for v in result['settling'].values())
    grade_path=folder/'grade.json'
    if grade_path.exists() and summary.get('variant') in ('faceted-drop','printed-drop') and summary.get('recording'):
        from grade_candy_trial import grade
        saved=json.loads(grade_path.read_text())
        if saved.get('control'):
            control=folder.parent/saved['control']
            measured=grade(folder,control)
            if measured!=saved:raise ValueError(f'{folder}: stored candy grade diverges from replayed observations')
            result['task_score']=measured['task_score']
            result['full_task_demonstrated']=bool(result['all_videos_valid'] and measured['full_task_demonstrated'])
            result['paired_grade']='grade.json'
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folders',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=[audit(p) for p in args.folders]
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    for r in result:
        print(json.dumps({k:r.get(k) for k in ('task','native_status','all_videos_valid','all_materials_candidate_settled','settling')}))
