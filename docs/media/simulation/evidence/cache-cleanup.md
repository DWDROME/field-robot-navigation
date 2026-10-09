# Task-local cache cleanup

The final 27-run matrix and RViz supplement are preserved under this evidence directory. Fifteen reviewed PNG/JPG pairs and metadata are one level above. Excluded diagnostic runs were also copied and 823 files individually hash-matched; ten additional host-side diagnostic files were copied and hash-matched into `diagnostics/host-checks/`. Build/source/geometry provenance and regression logs are preserved independently of `.tmp`.

The task container `sim-matrix-candidate` has been stopped after recording zero cgroup OOM events and no live Gazebo world. This worktree's `.tmp` contains only the `simulation` directory and the ten files listed below, all created for this change. It holds approximately 2.9 GiB, predominantly rebuildable task-specific sources, staged build products and duplicate run outputs. It contains no repository source or formal Comet state. The main workspace's `.tmp` was excluded from the initial baseline copy and is outside this cleanup.

```text
simulation/
candidate-air-qa3.log
candidate-ground-qa4.log
candidate-installed-final.json
far-free-terrain-overlay-final.log
far-free-terrain-overlay-reviewed-rviz.log
far-free-terrain-overlay.log
final-ground-ownership-proof.json
ground-grid-proof-r2.json
ground-grid-proof.json
navigation-inputs-qa4.json
```

Automatic approval rejected two prior recursive deletion attempts for obsolete build subdirectories and a later batch of individually hash-matched duplicate files, reporting only `blocked by policy`. Those rejected commands did not run. No alternative shell or deletion technique was used to bypass the rejection. Four earlier individually named duplicate logs were successfully removed after preserved-copy SHA256 comparison.

The remaining external cleanup is limited to this exact worktree cache:

```powershell
$taskCache = 'D:\test_move3d\.worktrees\gazebo-sim-matrix\.tmp'
$evidenceRoot = 'D:\test_move3d\.worktrees\gazebo-sim-matrix\docs\media\simulation\evidence'
foreach ($name in @('candidate-matrix\matrix_results.json', 'matrix-validation.json', 'diagnostics\inventory.json')) {
    if (-not (Test-Path -LiteralPath (Join-Path $evidenceRoot $name))) {
        throw "Missing preserved evidence: $name"
    }
}
if ((Resolve-Path -LiteralPath $taskCache).Path -ne $taskCache) {
    throw 'Unexpected cache path'
}
Remove-Item -LiteralPath $taskCache -Recurse -Force
```

The main workspace `D:\test_move3d\.tmp`, other changes, repository source, media, Comet state and Git history are outside this cleanup. Rebuilding remains possible through documented preparation/build scripts; the frozen simulation image retains all runtime assets. Comet Builder submission remains pending until required cleanup and final acceptance self-review are complete.
