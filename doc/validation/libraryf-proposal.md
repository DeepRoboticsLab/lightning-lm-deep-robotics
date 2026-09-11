> Archived proposal for historical context. Later user instructions authorized the work and changed its scope to seven-dataset qualification and publication. This is not a current task list or an approval gate. Proposed experiments and repetition counts are not completion claims; consult ../validation.md for completed evidence.

# Libraryf reconstruction improvement plan

Status: proposed for review on 2026-09-10. No experiments, builds, or reconstruction changes are authorized by this planning document itself or have been executed while writing it.

Target branch: `ysc-20260909`. Reference baseline from the prior review: `0d38bbc`; check the actual working tree and HEAD before starting, preserving any later user work. Reference image: `/home/ubuntu/Pictures/result.jpg`.

## 1. What success must look like

Recover the coherent building shape and revisit alignment visible in the reference: straight static walls, sensible near-right-angle building corners, matching opposite walls, and repeated passes that reinforce one surface instead of producing displaced copies. The optimized trajectory must be spatially consistent at revisits without implausible corrections. A visually smooth trajectory alone is insufficient; real turns must remain and map geometry must improve with it.

The photograph is a qualitative target, not surveyed ground truth. Do not infer exact dimensions or pixel-space angles from a perspective photograph. If the original reference cloud or configuration becomes available, incorporate it without blocking initial evaluation.

## 2. Preserve work and make experiments reproducible

- Keep development and historical comparisons in isolated worktrees. Do not switch or overwrite the user's working tree during a run.
- Preserve existing clouds, logs, reference image, and build compatibility changes. Give every execution a unique output directory; account for the application's relative `data/new_map` output before running anything.
- Put evaluation tools, reports, and generated artifacts under `/home/ubuntu/deep-robotics/reconstruction-evaluation/`, outside the public README. Small reusable instrumentation inside the repository must be isolated and reviewable.
- For every run save the source commit and patch, build identity, exact YAML, effective runtime parameters, bag identity/checksum, start/end times, thread policy, UI mode, exit status, processing counts, warnings, and output checksums.
- Process runs sequentially under comparable load. Use an explicitly built executable from the intended worktree and isolate ROS overlays so a historical comparison cannot accidentally run the current binary.
- Keep sensor topics, units, extrinsics, timestamp handling, and input filtering fixed during the first backend experiments. Never change calibration merely to make the picture look square.

## 3. Build the evaluation pipeline before tuning SLAM

### 3.1 Export the information needed to explain a bad map

Record full-precision timestamped raw LIO poses, raw keyframe poses, and final optimized poses with stable keyframe IDs. Capture optimized keyframe snapshots around important loop events; distinguish final corrected history from the pose that was available online at the time.

Record loop endpoints, candidate-generation order, registration scores and transforms, available convergence/overlap diagnostics, acceptance or rejection reason, residuals before and after optimization, edge level and actual active status. Record graph optimization times and finalization state. Retain existing counts of dropped or reversed timestamps.

Export local keyframe clouds or point-to-keyframe provenance if necessary for visit-specific overlap analysis. A single merged PCD cannot reliably identify which pass produced a duplicate wall. Scope this export to selected regions if full provenance is too expensive.

Verify instrumentation by comparing instrumented and uninstrumented baseline outputs within measured repeatability. Buffer diagnostic output where possible: logging can change scheduling, so it cannot simply be assumed inert.

### 3.2 Produce the same visual report for every run

Save an interactive 3D scene plus fixed PNG views and a compact comparison report. Use a reproducible local viewer selected after checking installed tools; static images must remain usable without that viewer.

Required views:

1. Whole map from above, showing coverage and global distortion.
2. Whole map obliquely, showing height structure and doubled surfaces.
3. Building close-up with each diagnostically useful wall and corner visible.
4. Return region close-up, with the first visit and return visit colored differently.
5. Trajectory-only view: raw and optimized trajectories separately selectable, colored by time, with loop endpoints and correction magnitudes.
6. Before/after snapshots around the first significant wrong correction, plus an end-of-run view.

Show reference, baseline and candidate together. Lock camera, crop, point size, height slab, color scale and rendering resolution. If display downsampling is needed, use the same voxel size and deterministic selection for every run, and keep full outputs. Save render settings and region selections as data rather than relying on manual camera positioning each time.

Do not smooth trajectories, snap walls to right angles, selectively hide bad regions, or choose only a favorable repeat. Include representative and worst runs. First inspect saved final results with UI disabled during processing, then check whether enabling the live UI changes the outcome.

### 3.3 Align comparisons without hiding distortion

Remove only one global rigid transform for each whole-run comparison. Prefer corresponding early stable keyframes or a predetermined stable region; record the chosen correspondence set and transform. Apply that same transform to the map and all trajectories. Never independently align each wall, scale a map, or warp a trajectory for the comparison images.

Separate comparison alignment from diagnostic local registration. If independently registering two revisit submaps to estimate their mismatch, report the required correction explicitly and retain the uncorrected overlay. Diagnostic registration must not silently repair the displayed result.

### 3.4 Measure geometry and revisit consistency

Freeze building wall regions and revisit time intervals after baseline inspection, before judging candidates. Select actual static surfaces; separate wall height bands from foliage, roof edges, furniture and moving objects. Report point support and coverage, not just a fitted error.

| Criterion | Measurement | Failure it should reveal |
| --- | --- | --- |
| Wall shape | Robust line/plane residual spread and residual versus distance along each wall | Thick walls, bowed walls, or a good short fit hiding a distorted long wall |
| Building corners | Angles between independently fitted adjacent wall directions | Departure from expected near-right-angle structure |
| Opposite walls | Direction difference and separation consistency along the walls | Shear, tapering, and inconsistent building width |
| Repeated surfaces | Visit-colored overlay, duplicate-surface separation, supported overlap residuals | Two passes failing to occupy the same physical surface |
| Revisit trajectory | Relative translation/yaw disagreement over corresponding revisit segments | Uncorrected return drift or an incorrect closure |
| Correction behavior | Keyframe corrections and abrupt spatial changes near loop events | A graph correction damaging previously good geometry |
| Repeatability | Median, range and worst-run geometry/closure results after global alignment | A lucky single reconstruction |
| Coverage and cost | Processed scans/keyframes, spatial coverage, wall support, runtime, peak memory | Apparent improvement caused by discarding difficult data or unacceptable resource cost |

Do not assume the first and last timestamps correspond to exactly the same physical position. Revisit segment selection must use scene evidence and remain unchanged for comparisons. Do not report ground-truth trajectory error without ground truth. Loop residuals are internal diagnostics and cannot independently prove a correct loop.

Check evaluation scripts on simple generated geometry: rigid transforms must not alter shape scores, a known shear must alter angles, a doubled wall must increase separation/thickness, and missing wall points must reduce reported support. These tests validate the evaluator, not SLAM quality.

## 4. Establish the baseline and locate when the damage occurs

Run at least three complete repetitions of the current implementation using the same setup. Existing runs provide evidence, but need fresh runs with the agreed exports for a fair comparison. If three runs show conflicting outcomes or one severe failure, expand to five before ranking small improvements.

Inspect the entire scene before reading aggregate scores. Identify the worst wall, the troublesome return segment, and the earliest keyframe or loop event where geometry starts to diverge. Compare local raw geometry with optimized geometry at that moment.

Write a short diagnosis after inspection:

- If local walls are already curved before loop correction, prioritize frontend/deskew/calibration evidence.
- If local walls are good and bend after a loop event, prioritize loop correspondence and graph behavior.
- If repetitions diverge before any loop, revisit frontend scheduling and input handling instead of assuming a backend-only cause.
- If only the live trail looks open while saved corrected geometry aligns, fix the display distinction without claiming a map improvement.

Use baseline variability to set practical acceptance tolerances before evaluating candidates. Record both major targets separately: building shape and return alignment. Do not hide a regression in one behind an averaged score.

Milestone deliverable: reference/baseline comparison images, baseline spread, selected regions, and a revised evidence-based experiment order.

## 5. Experiment queue: one hypothesis at a time

The order below is provisional. Baseline and each trial may change it; document the evidence when reordering. A negative result is useful if it rules out a cause. Do not accumulate unsuccessful modifications.

### E1. Recover the historical known-good candidate

Reproduce `f41860c` with its own library-adapted `config/default_nclt.yaml`. Apply only necessary, recorded build compatibility changes. Start with one full diagnostic run; if plausible, obtain at least three repetitions. If needed compare the March master library fixes as a second distinct historical experiment.

This is a historical code/configuration pair, not a single-variable causal experiment. If it resembles the reference substantially better, narrow the responsible changes with controlled configuration comparisons or a small commit bracket. Do not port the whole historical branch immediately or attribute success to one change without isolation.

### E2. Make rejected loop edges actually inactive

First verify the proposed solver correction on a small graph containing a contradictory constraint: after rejection, active edge counts, cost and solved geometry must reflect its removal. Account for incremental matrix bookkeeping. Then test the libraryf effect.

Separate correct exclusion from the additional solve after rejection where practical, so their effects are observable. Inspect whether previously rejected constraints influenced later poses and whether removing them repairs the specific warped building/return region.

### E3. Diagnose repeatability and the graph coordinate frame

Test serial backend assembly alone, leaving the frontend and acceptance policy unchanged. Compare the first divergent loop decisions across repeated runs. Then separately assess an appropriate pose prior or supported anchored solution if gauge behavior is implicated. Do not use unsupported fixed vertices in the incremental solver.

A deterministic wrong map is not a reconstruction improvement. A determinism change may be retained only as an explicit diagnostic mode unless it improves robustness or has another clearly justified benefit accepted in the plan.

### E4. Test graph optimization timing

After active-edge handling is trustworthy, test final graph refinement, then per-keyframe optimization as separate changes. Examine whether the trajectory and wall shape improve or whether repeated optimization amplifies a false loop. Compare runtime as well as geometry. Consider periodic optimization only if the evidence supports that tradeoff.

### E5. Improve loop validation where observed false matches justify it

Inspect the actual point-cloud overlap for problematic loop pairs. Start with invalid/non-finite and convergence checks. Add an overlap or consistency gate only after inspecting true and false candidates; test each policy separately. Treat robust weighting and rejection threshold policy as separate experiments.

Do not improve apparent stability by suppressing all useful loops. Confirm that the genuine return match is retained and that its geometry agrees. Inspect the effect on repetitive building facades before adjusting candidate radius or score thresholds.

### E6. Test active keyframe settings

First wire supported parameters while preserving existing effective defaults. Then compare the intended 5-degree keyframe threshold against the effective 15-degree baseline as its own experiment. Track changed keyframe density, geometry and runtime. Do not interpret edits to unsupported YAML switches as working algorithm changes.

### E7. Compare the newer frontend coherently

If raw geometry remains a limiting factor, bring forward this experiment. Otherwise evaluate it after establishing a reliable backend. Use compatible navigation state, filter, measurement model, map update logic and configuration together from the reviewed newer source. Keep sensor calibration and message conventions appropriate for M20.

Test with optional ICP disabled first; enable it separately afterward. A coherent frontend port is necessarily a larger experimental unit: if it wins, determine whether a smaller safe subset can preserve the gain without mixing incompatible state dimensions or interfaces.

### E8. Investigate input timing only if the evidence points there

Keep current dropped/reversed timestamp behavior fixed initially. If distortion correlates with timing discontinuities or raw deskew errors, inspect bag timestamps and sensor headers, then design a separate justified correction. Do not reorder data or change IMU conventions speculatively.

## 6. The decision loop after every experiment

1. State the hypothesis, exact change, expected visible effect, and possible regression before running.
2. Perform the narrow correctness check needed for that change; build only the necessary targets.
3. Run the full libraryf sequence once and generate the standard report.
4. Inspect whole-map and region views, trajectories, and the relevant loop-event snapshots. Use metrics to substantiate what is visible and investigate disagreement between them.
5. If promising, complete at least three comparable repetitions. If inconclusive, gather the specific missing evidence instead of declaring success or piling on another patch.
6. Record what changed, what did not, what was learned, and the next hypothesis. Update the experiment order when evidence contradicts the original explanation.
7. Keep the change only if it meets its stated goal beyond baseline variation and does not materially damage the other reconstruction targets. Revert an ineffective or harmful experimental algorithm change; retain its artifacts for learning.

Compare each candidate with both the original baseline and the last accepted candidate. Distinguish instrument-only or diagnostic changes from reconstruction improvements. Preserve useful evaluation tools without presenting them as algorithm gains.

## 7. Final acceptance and rollback

- Obtain five full repetitions of the combined winning candidate. Inspect the worst result as well as a representative one. Do not select the best-looking run as the final evidence.
- Require visibly improved return alignment and building geometry, supported by the frozen measurements and preserved coverage. An intermediate patch may fix one issue without worsening the other; the final result must address both user-visible problems.
- If baseline variability makes the gain ambiguous, add targeted repetitions or reject the improvement claim. Do not present small-sample variation as a statistical guarantee.
- Run one comparison with live visualization enabled and check its saved result against the offline distribution. Make raw and corrected trajectories clearly distinguishable in the viewer.
- Use another suitable downloaded bag as a regression check. Verify Humble build/runtime locally; verify Foxy ARM natively when access is available and report it as pending until then.
- Review accepted changes individually and as a combination; remove redundant or ineffective patches. Preserve the established compatibility changes and all user work.
- Deliver before/after images, an interactive viewing command, repeatability results, exact rerun commands, and a concise explanation of why each retained change helps. Keep experiment journals outside the public README; document only lasting general usage/setup changes there.

Rollback applies only to experiment-owned changes. Failed outputs and notes remain in their separate run directories. If none of the candidates meets the target, leave the branch with its original reconstruction behavior and report the remaining evidence and next justified investigation.

## 8. Review checkpoints

Provide a visual update after the baseline, after historical reproduction, after each meaningful candidate, and at final qualification. Each update should answer: where is the defect, what changed visually, what supports that interpretation, and what will be tried next?

The intended workflow is sustained empirical investigation with bounded, reversible changes. It does not assume the source review already identified the winning fix, and it does not claim the reference quality has been recovered until the saved outputs demonstrate it repeatedly.
