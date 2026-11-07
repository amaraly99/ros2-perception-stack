# ros2-perception-stack (branch refactor/pre-main)

Clean-slate branch. It starts empty on purpose: files are brought in one at a time
from `main`, `sim/unified-start-2026-10-04` and `exp/detector-accuracy`, and each one
is placed in its target folder as it arrives.

## Rules
- Bring files in with `git checkout <branch> -- <path>`, then `git mv` to the target folder. One topic per commit.
- Do not copy vendored SLAM sources, results, plots or Simulink caches (`slprj/`). They stay out of git.
- Claude never commits or pushes. Amar does all commits and pushes himself; Claude edits files and hands him the commands.
- Never add `Co-Authored-By: Claude` or any AI attribution to commits or PRs.
- Never force-push, delete branches or delete `paper-batch-*` tags.
- No hardcoded IPs, SSH hosts or Windows paths. They go in `config/site.yaml` (git-ignored).
- Target layout and migration notes: `docs/README.md`.
