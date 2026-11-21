# Sokoban solver — make.fun sokoban-solver-01-run-20261121

Written from scratch for this bounty. It is not a copy of the kit starter
or the plain-astar baseline, and it uses no Festival / Sokolution /
Takaken code.

## Idea

The generator builds a level as a one-cell hallway plus one-door rooms
of 3–4 boxes each, and a box pushed out of a room can never come back,
so the push-optimal total is the sum of the rooms' push-optima. The
program splits a level at its hallway, solves each room independently
with a push-based A* (one step = one push, cost = pushes), and stitches
the room solutions together with plain walks in the hallway.

The A* heuristic is the minimum-cost assignment of boxes to goals
under shortest-path box distances, which is admissible for pushes.
It prunes boxes that can never be pushed anywhere (walls alone
blocking them). All budgets are counted in expansions, never in
wall-clock time, so two runs write the same bytes.

## Run

```
node main.ts
```

Reads `/input/<id>/level.txt`, writes `/output/<id>/solution.txt`.
Locally: `MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts`
with the kit's practice set.

## Practice results

Official `grade-local --driver process` on the kit's practice set,
2026-11-21: **score 1,000,000** (all five instances at push-optimal).

| instance | P* pushes | our pushes | points |
| --- | --- | --- | --- |
| i001 | 13 | 13 | 1,000 |
| i002 | 16 | 16 | 1,000 |
| i003 | 29 | 29 | 1,000 |
| i004 | 47 | 47 | 1,000 |
| i005 | 60 | 60 | 1,000 |

House baseline plain-astar scores 200,000 on the same practice set
(it solves only i001). Total local runtime is about 0.5 s for the
whole set, well inside the 60 s wall budget. Two runs write identical
outputs (verified by diff).

## Generalisation check

Three fresh sets drawn with the kit's own generator (`program/
generator/draw.ts`) from unrelated seeds, scored against their
manifest P* values: all three score **1,000,000**, every instance
push-optimal (fresh1: P* 16/21/30/44/60; fresh2: 14/15/33/42/59;
fresh3: 13/18/31/44/69), each set in under 0.5 s. The solver is not
fitted to the practice set.
