# Independent-reset failure and correction

During the pre-fix matrix, the sixth ground run lost Gazebo with exit 137. The container's cgroup reported `oom 1`, `oom_kill 1`, and 2147483648 bytes of swap usage after the kill. Host memory still had approximately 25 GiB available; the bounded task container exhausted its own allocation.

`docker top` showed a surviving prior-run server: host PID 554459, parent 542046 (the container's `sleep infinity`), process group 554366, RSS 1696992 KiB. Its command referenced `maize_field_husky_a200_r2/maize_field-runtime.sdf` while the runner was executing r3. The launch log said the Ruby Gazebo wrapper had terminated, but the actual server remained. Signalling only the launch process group did not establish an independent reset.

The runner now captures the launch's descendant tree and the server carrying this run's exact world-file argument before shutdown. After graceful launch shutdown, it terminates those processes and uses SIGKILL only for remaining owned processes. `/proc` start times guard against PID reuse. Each result records tracked PIDs, forced kills and remaining PIDs. A remaining process blocks the result and stops the matrix before another world starts. This correction does not increase memory limits or relax navigation safety.

Separately, the first orchard attempt reported `Switch controller timed out after 5 seconds!`. The installed controller-manager spawner documents distinct availability, service-response and switch timeouts. The SIM launch now gives startup service and switch operations 120 wall seconds, preserving the separate physical settling and navigation freshness checks.

Pre-fix records are diagnostic and are excluded from the final independently reset matrix. The process lifecycle regression uses an actual reparented child in another session, forces its termination and verifies an unrelated sibling survives. Its result and the final matrix cleanup records provide the correction's verification evidence.
