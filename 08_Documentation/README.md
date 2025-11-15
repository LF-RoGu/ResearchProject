# 08_Documentation

Everything you need to explain or publish the project lives here—from the LaTeX source files to the compiled PDF and supporting
figures. Start with this overview before editing the report so you know which assets to touch.

## Quick tour

| Item | Description |
| --- | --- |
| `RadarOdometry.pdf` | Latest compiled version of the written report. |
| `main.tex` | Master LaTeX file that stitches together every section under `section/`. |
| `section/` | Individual chapter files (introduction, methodology, experiments, etc.). |
| `images/` | All figures referenced in the LaTeX sources and the root README (e.g., `labDriveAroundICP_Full1.png`). |
| `Bibliography.tex` | Shared BibTeX entries cited throughout the report. |
| `ProjectNavigationGuide.md` | Markdown orientation guide for contributors who prefer a lightweight reference. |

## Editing guidelines

1. Update figures or plots inside `images/` first, then recompile `main.tex` to confirm labels and references resolve correctly.
2. Keep new sections modular by adding a file under `section/` and including it from `main.tex` to minimize merge conflicts.
3. When exporting visuals for the README, store them in `09_Images/` and (if relevant) copy the finalized versions into `08_Documentation/images/` for archival purposes.
