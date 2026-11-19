# Evidence provenance

The task source files are byte-identical to the accepted runtime. Demo geometry is the explicitly sampled30-of267-state export, with a continuous score trace. Current source snapshots remain available for the audit.

The model EvalLog is a submission copy: only `stats.frames_dir` and `stats.records_dir` were converted from absolute machine paths to relative paths. No actions, scores, timestamps, policy responses or termination fields were altered. Descriptive machine paths in demo verification metadata were anonymised; recorded source content/hash pairs were preserved. Original files and hashes remain in the development archive.

The submission model video is an untrimmed, synchronised layout of the recorded top and two wrist-camera streams, preserving all3601 frames at20 Hz. The demo is copied byte-for-byte from the verified66.6-second overview recording. The setup image is a native simulator view. Media is uploaded separately and referenced by the actual returned media ID.

The derived model EvalLog is stored as `model_eval.json.gz` using lossless gzip compression. Its decompressed SHA-256 is `6eb3b5f9bbecd28d54d4be9b64cf19f80f7a474725dc4c562cc055d84f23bd23`; a byte-for-byte round-trip check passed before submission.
