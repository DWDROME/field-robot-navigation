"""Prove escaped simulation children die while unrelated siblings survive."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import pytest

path=Path(__file__).parents[1]/'scripts/run_simulation_run.py'
spec=importlib.util.spec_from_file_location('simulation_runner',path)
runner=importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


@pytest.mark.parametrize('process_title', [False, True])
def test_cleanup_owns_reparented_world_process_and_preserves_sibling(tmp_path, process_title):
    marker=tmp_path/'world-runtime.sdf'
    pid_file=tmp_path/'child.pid'
    child_code=('import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); '
                'signal.signal(signal.SIGINT,signal.SIG_IGN); time.sleep(120)')
    parent_code=('import subprocess,sys,pathlib,time; '
        'p=subprocess.Popen([sys.executable,"-c",sys.argv[1],sys.argv[2]],start_new_session=True); '
        'pathlib.Path(sys.argv[3]).write_text(str(p.pid)); time.sleep(.3)')
    unrelated=subprocess.Popen([sys.executable,'-c',child_code,str(marker)+'-unrelated'],
                               start_new_session=True)
    world_argument=f'gz sim -r -s --headless-rendering {marker}' if process_title else str(marker)
    parent=subprocess.Popen([sys.executable,'-c',parent_code,child_code,world_argument,str(pid_file)],
                            start_new_session=True)
    child_pid=child_state=None
    try:
        parent.wait(timeout=5)
        child_pid=int(pid_file.read_text())
        tracked=runner.simulation_processes(parent.pid,marker)
        assert child_pid in tracked
        assert unrelated.pid not in tracked
        child_state=tracked[child_pid]
        result=runner.stop_simulation(parent,marker)
        assert child_pid in result['sigkill_pids']
        assert result['remaining_pids']==[]
        assert unrelated.poll() is None
    finally:
        for process in [parent,unrelated]:
            if process.poll() is None:
                process.kill()
            process.wait(timeout=5)
        if child_pid is not None and child_state is not None and runner.process_alive(child_pid,child_state):
            try:
                # Only the child created by this test can be cleaned here.
                os.kill(child_pid,signal.SIGKILL)
            except ProcessLookupError:
                pass
