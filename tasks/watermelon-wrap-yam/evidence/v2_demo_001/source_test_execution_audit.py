import hashlib,json,tempfile,unittest
from pathlib import Path
from verify_execution import audit_sources,audit_episode_contract,REQUIRED_SOURCES

class SourceAuditTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)/'reviewed';self.root.mkdir()
        self.run=Path(self.tmp.name)/'episode';self.run.mkdir()
        self.hashes={}
        for name in sorted(REQUIRED_SOURCES|{'verify_execution.py','test_old.py'}):
            data=('# original '+name+'\n').encode()
            (self.root/name).write_bytes(data)
            self.snapshot(name).parent.mkdir(parents=True,exist_ok=True)
            self.snapshot(name).write_bytes(data)
            self.hashes[name]=hashlib.sha256(data).hexdigest()
        self.manifest()
    def test_matching_startup_snapshot_passes(self):
        self.assertTrue(audit_sources(self.run,self.root)['verified'])
    def test_tampered_snapshot_fails_even_when_current_source_matches_manifest(self):
        self.snapshot('arena_task.py').write_text('# altered scene')
        report=audit_sources(self.run,self.root)
        self.assertFalse(report['verified']);self.assertTrue(any('snapshot' in x for x in report['failures']))
    def test_old_run_cannot_validate_changed_scorer(self):
        (self.root/self.scorer).write_text('# revised scoring')
        report=audit_sources(self.run,self.root)
        self.assertFalse(report['verified']);self.assertTrue(any('audited runtime source: '+self.scorer in x for x in report['failures']))
    def test_removed_required_manifest_entry_fails(self):
        del self.hashes[self.scorer];self.manifest()
        self.assertFalse(audit_sources(self.run,self.root)['verified'])
    def test_auditor_and_test_edits_do_not_change_runtime_identity(self):
        for name in ['verify_execution.py','test_old.py']:(self.root/name).write_text('# newer offline check')
        self.assertTrue(audit_sources(self.run,self.root)['verified'])
    def test_missing_manifest_fails_closed(self):
        self.manifest_path.unlink()
        self.assertFalse(audit_sources(self.run,self.root)['verified'])
    def test_manifest_path_escape_rejected(self):
        self.hashes['../outside.py']='0'*64;self.manifest()
        self.assertFalse(audit_sources(self.run,self.root)['verified'])

    scorer='scoring.py'
    def snapshot(self,name):return self.run/('source_'+name)
    @property
    def manifest_path(self):return self.run/'manifest.json'
    def manifest(self):self.manifest_path.write_text(json.dumps({'sources':self.hashes}))

class EpisodeContractTests(unittest.TestCase):
    def test_exact_action_limit_passes(self):
        self.assertEqual(audit_episode_contract([dict(step=0)],dict(step=3600),3600),[])
    def test_excess_final_or_trace_actions_fail(self):
        self.assertTrue(audit_episode_contract([dict(step=0)],dict(step=3601),3601))
        self.assertTrue(audit_episode_contract([dict(step=3601)],dict(step=3600),3600))
    def test_controller_and_scorer_step_counts_must_agree(self):
        self.assertTrue(audit_episode_contract([dict(step=10)],dict(step=15),20))
    def test_missing_or_invalid_count_fails(self):
        self.assertTrue(audit_episode_contract([dict(step=0)],dict(step=20),None))
        self.assertTrue(audit_episode_contract([dict(step=0)],dict(step=-1),-1))

if __name__=='__main__':unittest.main()
