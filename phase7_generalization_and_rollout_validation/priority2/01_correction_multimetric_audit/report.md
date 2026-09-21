# Correction Action Chunk Audit

Stored correction outputs were inspected before recomputation.

Result: existing correction artifacts mainly store L2/component metric summaries, not corrected frame-level action chunks. Therefore full L1/RMSE/Huber/cosine recomputation for those exact correction methods requires action-head rerun or saved corrected chunks. Priority2 reran train-fold hidden alignment baselines through the action head where feasible and marked older metric-only correction artifacts accordingly.
