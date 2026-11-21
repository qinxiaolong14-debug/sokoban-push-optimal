// Sokoban solver for make.fun sokoban-solver-01-run-20261121.
// Written from scratch for this bounty (not a copy of the kit starter or the
// plain-astar baseline, and not Festival / Sokolution / Takaken code).
//
// Idea: the generator builds a level as a one-cell hallway plus one-door
// rooms of 3-4 boxes each, and a box pushed out of a room can never come
// back, so the push-optimal total is the sum of the rooms' push-optima.
// This program therefore splits a level at its hallway, solves each room
// independently with a push-based A* (one step = one push, cost = pushes),
// and stitches the room solutions together with plain walks in the
// hallway. The A* heuristic is the minimum-cost assignment of boxes to
// goals under shortest-path box distances, which is admissible for pushes,
// and it prunes boxes that can never be pushed anywhere (walls alone
// blocking them). All budgets are counted in expansions, never in
// wall-clock time, so two runs of the program write the same bytes.
//
// Run: node main.ts, reading /input/<id>/level.txt and writing
// /output/<id>/solution.txt; MAKEFUN_INPUT_DIR / MAKEFUN_OUTPUT_DIR
// stand in for /input and /output in a local run.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const LETTER = ['u', 'd', 'l', 'r'];
// Deterministic work budgets (expansions, not time): a room that needs
// more than this is given up on rather than risking the 60 s wall budget.
const ROOM_EXPANSIONS = 200000;

type Grid = {
  w: number;
  h: number;
  wall: boolean[];
  goal: boolean[];
  boxes: number[];
  player: number;
};

function parseLevel(text: string): Grid {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const wall: boolean[] = new Array(w * h).fill(false);
  const goal: boolean[] = new Array(w * h).fill(false);
  const boxes: number[] = [];
  let player = 0;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      wall[i] = ch === '#';
      if ('.*+'.includes(ch)) goal[i] = true;
      if ('$*'.includes(ch)) boxes.push(i);
      if ('@+'.includes(ch)) player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goal, boxes, player };
}

// BFS distances from `start` over cells where passable[i] is true.
// Returns an Int32Array of distances, -1 for unreachable.
function bfsDist(start: number, passable: boolean[], w: number): Int32Array {
  const dist = new Int32Array(passable.length).fill(-1);
  const q: number[] = [start];
  dist[start] = 0;
  for (let head = 0; head < q.length; head++) {
    const c = q[head]!;
    const cx = c % w;
    const cy = Math.floor(c / w);
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d]!;
      const ny = cy + DY[d]!;
      if (nx < 0 || ny < 0) continue;
      const n = ny * w + nx;
      if (n >= passable.length || !passable[n] || dist[n] >= 0) continue;
      dist[n] = dist[c]! + 1;
      q.push(n);
    }
  }
  return dist;
}

// Cells reachable by walking from `start` without stepping on a box.
// `blocked` marks box cells. Returns a Uint8Array visited mask.
function reachableMask(start: number, passable: boolean[], blocked: boolean[], w: number): Uint8Array {
  const seen = new Uint8Array(passable.length);
  if (!passable[start] || blocked[start]) return seen;
  const q: number[] = [start];
  seen[start] = 1;
  for (let head = 0; head < q.length; head++) {
    const c = q[head]!;
    const cx = c % w;
    const cy = Math.floor(c / w);
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d]!;
      const ny = cy + DY[d]!;
      if (nx < 0 || ny < 0) continue;
      const n = ny * w + nx;
      if (n >= passable.length || !passable[n] || blocked[n] || seen[n]) continue;
      seen[n] = 1;
      q.push(n);
    }
  }
  return seen;
}

// Shortest walk (list of direction indices) from `from` to `to` over
// passable cells avoiding boxes, or null. Fixed direction order, so
// the result is deterministic.
function walkPath(from: number, to: number, passable: boolean[], blocked: boolean[], w: number): number[] | null {
  if (from === to) return [];
  const prev = new Int32Array(passable.length).fill(-1);
  const prevDir = new Int8Array(passable.length).fill(-1);
  const q: number[] = [from];
  prev[from] = from;
  for (let head = 0; head < q.length; head++) {
    const c = q[head]!;
    const cx = c % w;
    const cy = Math.floor(c / w);
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d]!;
      const ny = cy + DY[d]!;
      if (nx < 0 || ny < 0) continue;
      const n = ny * w + nx;
      if (n >= passable.length || !passable[n] || blocked[n] || prev[n] >= 0) continue;
      prev[n] = c;
      prevDir[n] = d;
      if (n === to) {
        const path: number[] = [];
        let cur = n;
        while (cur !== from) {
          path.push(prevDir[cur]!);
          cur = prev[cur]!;
        }
        path.reverse();
        return path;
      }
      q.push(n);
    }
  }
  return null;
}

type Push = { box: number; dir: number };
type PushProblem = {
  w: number;
  passable: boolean[]; // cells the player may stand on
  boxCells: boolean[]; // cells a box may be pushed onto
  boxes: number[];
  goals: number[];
  startPlayer: number;
  door: number; // -1: no exit requirement
  requireExit: boolean;
  maxExpansions: number;
};

// Static dead cells for a push problem: a non-goal cell a box can never
// leave (corner, or a wall segment with no goal on it).
function deadCells(p: PushProblem): boolean[] {
  const { w, boxCells } = p;
  const n = boxCells.length;
  const dead = new Array<boolean>(n).fill(false);
  const solid = (i: number): boolean => i < 0 || i >= n || !boxCells[i];
  const at = (x: number, y: number): number => y * w + x;
  // Immobile box: no push is possible at all from this cell — every
  // direction either has no box cell to move into, or no cell the player
  // can stand on to push from. (A box on the door cell is NOT dead:
  // the player stands in the hallway entry and pushes it in.)
  for (let i = 0; i < n; i++) {
    if (!boxCells[i] || p.goals.includes(i)) continue;
    const x = i % w;
    const y = Math.floor(i / w);
    let canMove = false;
    for (let d = 0; d < 4; d++) {
      const dest = at(x + DX[d]!, y + DY[d]!);
      const behind = at(x - DX[d]!, y - DY[d]!);
      if (dest >= 0 && dest < n && boxCells[dest] && behind >= 0 && behind < n && p.passable[behind]) {
        canMove = true;
        break;
      }
    }
    if (!canMove) dead[i] = true;
  }
  // Note: no wall-line marking here. A box sliding along a wall can
  // leave the wall at its end, so marking a whole wall segment dead
  // (no goal on it) is unsound — it wrongly kills boxes that can slide
  // off the wall's end, including the door box of the practice levels.
  void solid;
  return dead;
}

// Push-based A* over a PushProblem. Returns the push sequence (fewest
// pushes, by the admissible assignment heuristic) or null when the
// expansion budget runs out or no solution exists under it.
function solvePush(p: PushProblem): Push[] | null {
  const { w, passable, boxCells, goals } = p;
  const n = passable.length;
  const goalSet = new Set(goals);
  const dead = deadCells(p);
  // Shortest-path box distances to each goal, over box cells only.
  const distGoal: Int32Array[] = goals.map((g) => bfsDist(g, boxCells, w));
  const boxDist = (box: number, gi: number): number => distGoal[gi]![box] ?? -1;

  // Admissible heuristic: minimum total box-to-goal distance, assigning
  // distinct goals when there are few boxes, else the per-box minimum.
  const heuristic = (boxes: number[]): number => {
    const m = boxes.length;
    if (m === 0) return 0;
    if (m <= 6) {
      let best = Infinity;
      const used = new Array<boolean>(goals.length).fill(false);
      const rec = (i: number, acc: number): void => {
        if (acc >= best) return;
        if (i === m) {
          best = acc;
          return;
        }
        for (let j = 0; j < goals.length; j++) {
          if (used[j]) continue;
          const d = boxDist(boxes[i]!, j);
          if (d < 0) continue;
          used[j] = true;
          rec(i + 1, acc + d);
          used[j] = false;
        }
      };
      rec(0, 0);
      return best;
    }
    let sum = 0;
    for (const b of boxes) {
      let d = Infinity;
      for (let j = 0; j < goals.length; j++) {
        const v = boxDist(b, j);
        if (v >= 0 && v < d) d = v;
      }
      if (d === Infinity) return Infinity;
      sum += d;
    }
    return sum;
  };

  const blockedOf = (boxes: number[]): boolean[] => {
    const b = new Array<boolean>(n).fill(false);
    for (const c of boxes) b[c] = true;
    return b;
  };
  const repOf = (player: number, boxes: number[]): number => {
    const mask = reachableMask(player, passable, blockedOf(boxes), w);
    for (let i = 0; i < n; i++) if (mask[i]) return i;
    return -1;
  };
  const packKey = (boxes: number[], rep: number): number => {
    let k = rep + 1;
    for (const b of boxes) k = k * 256 + (b + 1);
    return k;
  };
  const isGoalBoxes = (boxes: number[]): boolean => boxes.every((b) => goalSet.has(b));
  // A state is deadlocked only when a non-goal box sits on a static
  // dead cell (it can never be pushed anywhere, walls alone blocking
  // it). A 2x2 test is NOT used: a full 2x2 that includes hallway or
  // entry cells does not freeze a box — the door box of the practice
  // levels is the counterexample, and a block of boxes can also open
  // up when another box moves away.
  const hasDeadlock = (boxes: number[]): boolean => {
    for (const b of boxes) {
      if (goalSet.has(b)) continue;
      if (dead[b]) return true;
    }
    return false;
  };

  const startBoxes = [...p.boxes].sort((a, b) => a - b);
  if (hasDeadlock(startBoxes)) return null;
  const h0 = heuristic(startBoxes);
  if (h0 === Infinity) return null;

  const sBoxes: number[][] = [startBoxes];
  const sPlayer: number[] = [p.startPlayer];
  const sG: number[] = [0];
  const sKey: number[] = [packKey(startBoxes, repOf(p.startPlayer, startBoxes))];
  const sParent: number[] = [-1];
  const sPushBox: number[] = [-1];
  const sPushDir: number[] = [-1];
  const best = new Map<number, number>([[sKey[0]!, 0]]);
  // Binary heap of [f, g, state], ordered by f, then larger g, then index.
  const heap: [number, number, number][] = [[h0, 0, 0]];
  const less = (a: [number, number, number], b: [number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] > b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: [number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const par = (i - 1) >> 1;
      if (!less(heap[i]!, heap[par]!)) break;
      [heap[i], heap[par]] = [heap[par]!, heap[i]!];
      i = par;
    }
  };
  const heapPop = (): [number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };

  let expanded = 0;
  while (heap.length > 0) {
    const [f, g, s] = heapPop();
    if (best.get(sKey[s]!) !== s || sG[s] !== g) continue;
    void f;
    const boxes = sBoxes[s]!;
    const player = sPlayer[s]!;
    if (isGoalBoxes(boxes)) {
      if (!p.requireExit || p.door < 0) {
        return reconstruct(s);
      }
      const mask = reachableMask(player, passable, blockedOf(boxes), w);
      if (mask[p.door]) return reconstruct(s);
      // Goal boxes but the player cannot get out: keep searching for a
      // goal state that can exit.
    }
    if (expanded >= p.maxExpansions) return null;
    expanded++;
    const mask = reachableMask(player, passable, blockedOf(boxes), w);
    const boxSet = new Set(boxes);
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi]!;
      const bx = b % w;
      const by = Math.floor(b / w);
      for (let d = 0; d < 4; d++) {
        const destX = bx + DX[d]!;
        const destY = by + DY[d]!;
        if (destX < 0 || destY < 0) continue;
        const dest = destY * w + destX;
        if (dest >= n || !boxCells[dest] || boxSet.has(dest)) continue;
        const behindX = bx - DX[d]!;
        const behindY = by - DY[d]!;
        if (behindX < 0 || behindY < 0) continue;
        const behind = behindY * w + behindX;
        if (behind >= n || !mask[behind]) continue;
        const nb = [...boxes];
        nb[bi] = dest;
        nb.sort((a, b2) => a - b2);
        if (hasDeadlock(nb)) continue;
        const h = heuristic(nb);
        if (h === Infinity) continue;
        const ng = g + 1;
        const nPlayer = b;
        const rep = repOf(nPlayer, nb);
        const key = packKey(nb, rep);
        const seen = best.get(key);
        if (seen !== undefined && sG[seen]! <= ng) continue;
        const t = sBoxes.length;
        sBoxes.push(nb);
        sPlayer.push(nPlayer);
        sG.push(ng);
        sKey.push(key);
        sParent.push(s);
        sPushBox.push(b);
        sPushDir.push(d);
        best.set(key, t);
        heapPush([ng + h, ng, t]);
      }
    }
  }
  return null;

  function reconstruct(s: number): Push[] {
    const pushes: Push[] = [];
    for (let t = s; t > 0; t = sParent[t]!) {
      pushes.push({ box: sPushBox[t]!, dir: sPushDir[t]! });
    }
    pushes.reverse();
    return pushes;
  }
}

// ---- Level solving: split at the hallway, solve each room, stitch ----

type Room = {
  cells: number[]; // room cells (including the door)
  door: number;
  entry: number; // hallway cell next to the door
  boxes: number[];
  goals: number[];
};

function findRooms(g: Grid): Room[] | null {
  const { w, h, wall, goal, boxes, player } = g;
  const hy = Math.floor(player / w);
  const inHallway = (i: number): boolean => Math.floor(i / w) === hy;
  // Connected components of floor cells off the hallway row.
  const seen = new Array<boolean>(w * h).fill(false);
  const rooms: Room[] = [];
  const boxSet = new Set(boxes);
  for (let i = 0; i < w * h; i++) {
    if (wall[i] || inHallway(i) || seen[i]) continue;
    const comp: number[] = [];
    const q: number[] = [i];
    seen[i] = true;
    for (let head = 0; head < q.length; head++) {
      const c = q[head]!;
      comp.push(c);
      const cx = c % w;
      const cy = Math.floor(c / w);
      for (let d = 0; d < 4; d++) {
        const nx = cx + DX[d]!;
        const ny = cy + DY[d]!;
        if (nx < 0 || ny < 0 || ny >= h) continue;
        const n = ny * w + nx;
        if (wall[n] || inHallway(n) || seen[n]) continue;
        seen[n] = true;
        q.push(n);
      }
    }
    const compSet = new Set(comp);
    const roomBoxes = boxes.filter((b) => compSet.has(b));
    const roomGoals = comp.filter((c) => goal[c]);
    if (roomBoxes.length === 0 && roomGoals.length === 0) continue;
    if (roomBoxes.length !== roomGoals.length) return null;
    // Door: the room cell adjacent to a hallway cell.
    const doors = comp.filter((c) => {
      const cx = c % w;
      const cy = Math.floor(c / w);
      for (let d = 0; d < 4; d++) {
        const nx = cx + DX[d]!;
        const ny = cy + DY[d]!;
        if (nx < 0 || ny < 0 || ny >= h) continue;
        const n = ny * w + nx;
        if (!wall[n] && inHallway(n)) return true;
      }
      return false;
    });
    if (doors.length !== 1) return null;
    const door = doors[0]!;
    const dx = door % w;
    const dy = Math.floor(door / w);
    let entry = -1;
    for (let d = 0; d < 4; d++) {
      const nx = dx + DX[d]!;
      const ny = dy + DY[d]!;
      if (nx < 0 || ny < 0 || ny >= h) continue;
      const n = ny * w + nx;
      if (!wall[n] && inHallway(n)) {
        entry = n;
        break;
      }
    }
    if (entry < 0) return null;
    rooms.push({ cells: comp, door, entry, boxes: roomBoxes, goals: roomGoals });
  }
  // Every box and every goal must belong to exactly one room, and the
  // player must start in the hallway.
  const roomBoxCount = rooms.reduce((a, r) => a + r.boxes.length, 0);
  const roomGoalCount = rooms.reduce((a, r) => a + r.goals.length, 0);
  if (roomBoxCount !== boxes.length || roomGoalCount !== boxes.length) return null;
  if (boxSet.has(player)) return null;
  rooms.sort((a, b) => a.door - b.door);
  return rooms;
}

// Expand a room's push sequence into walk+push moves, starting with the
// player at `entry` and the room's boxes as given. Returns the moves and
// the player's final cell, or null if a walk is impossible.
function expandRoom(
  g: Grid,
  room: Room,
  pushes: Push[],
  startPlayer: number,
): { moves: string; endPlayer: number } | null {
  const { w, wall } = g;
  const roomSet = new Set(room.cells);
  const passable = wall.map((isWall, i) => !isWall && (roomSet.has(i) || i === room.entry));
  const boxes = new Set(room.boxes);
  let player = startPlayer;
  const out: string[] = [];
  const blocked = (): boolean[] => {
    const b = new Array<boolean>(w * g.h).fill(false);
    for (const c of boxes) b[c] = true;
    return b;
  };
  for (const push of pushes) {
    const bx = push.box % w;
    const by = Math.floor(push.box / w);
    const behind = (by - DY[push.dir]!) * w + (bx - DX[push.dir]!);
    const dest = (by + DY[push.dir]!) * w + (bx + DX[push.dir]!);
    const walk = walkPath(player, behind, passable, blocked(), w);
    if (walk === null) return null;
    for (const d of walk) out.push(LETTER[d]!);
    out.push(LETTER[push.dir]!.toUpperCase());
    boxes.delete(push.box);
    boxes.add(dest);
    player = push.box;
  }
  // Walk out through the door to the hallway entry cell.
  const exitWalk = walkPath(player, room.door, passable, blocked(), w);
  if (exitWalk === null) return null;
  for (const d of exitWalk) out.push(LETTER[d]!);
  const dx = room.entry % w - (room.door % w);
  const dy = Math.floor(room.entry / w) - Math.floor(room.door / w);
  const exitDir = DX.findIndex((v, i) => v === dx && DY[i] === dy);
  if (exitDir < 0) return null;
  out.push(LETTER[exitDir]!);
  return { moves: out.join(''), endPlayer: room.entry };
}

function solveLevel(text: string): string {
  const g = parseLevel(text);
  const { w, h, wall, goal } = g;
  const n = w * h;
  const rooms = findRooms(g);
  const allPassable = wall.map((isWall) => !isWall);
  const globalBlocked = (boxes: number[]): boolean[] => {
    const b = new Array<boolean>(n).fill(false);
    for (const c of boxes) b[c] = true;
    return b;
  };
  if (rooms === null) {
    // Fallback: one push-A* over the whole level, no exit requirement.
    const allGoals: number[] = [];
    for (let i = 0; i < n; i++) if (goal[i]) allGoals.push(i);
    const pushes = solvePush({
      w,
      passable: allPassable,
      boxCells: allPassable,
      boxes: g.boxes,
      goals: allGoals,
      startPlayer: g.player,
      door: -1,
      requireExit: false,
      maxExpansions: ROOM_EXPANSIONS,
    });
    if (pushes === null) return '';
    // Expand pushes with whole-level walks.
    const boxes = new Set(g.boxes);
    let player = g.player;
    const out: string[] = [];
    for (const push of pushes) {
      const bx = push.box % w;
      const by = Math.floor(push.box / w);
      const behind = (by - DY[push.dir]!) * w + (bx - DX[push.dir]!);
      const dest = (by + DY[push.dir]!) * w + (bx + DX[push.dir]!);
      const walk = walkPath(player, behind, allPassable, globalBlocked([...boxes]), w);
      if (walk === null) return '';
      for (const d of walk) out.push(LETTER[d]!);
      out.push(LETTER[push.dir]!.toUpperCase());
      boxes.delete(push.box);
      boxes.add(dest);
      player = push.box;
    }
    return out.join('');
  }

  let player = g.player;
  const globalBoxes = new Set(g.boxes);
  const out: string[] = [];
  for (const room of rooms) {
    // Walk the hallway to this room's entry cell.
    const hallWalk = walkPath(player, room.entry, allPassable, globalBlocked([...globalBoxes]), w);
    if (hallWalk === null) return '';
    for (const d of hallWalk) out.push(LETTER[d]!);
    player = room.entry;
    // Solve the room, first requiring the player to be able to exit
    // afterwards, then without that requirement as a fallback.
    const roomSet = new Set(room.cells);
    const passable = wall.map((isWall, i) => !isWall && (roomSet.has(i) || i === room.entry));
    const boxCells = wall.map((isWall, i) => !isWall && roomSet.has(i));
    let pushes = solvePush({
      w,
      passable,
      boxCells,
      boxes: room.boxes,
      goals: room.goals,
      startPlayer: room.entry,
      door: room.door,
      requireExit: true,
      maxExpansions: ROOM_EXPANSIONS,
    });
    if (pushes === null) {
      pushes = solvePush({
        w,
        passable,
        boxCells,
        boxes: room.boxes,
        goals: room.goals,
        startPlayer: room.entry,
        door: room.door,
        requireExit: false,
        maxExpansions: ROOM_EXPANSIONS,
      });
    }
    if (pushes === null) return '';
    const expanded = expandRoom(g, room, pushes, room.entry);
    if (expanded === null) return '';
    out.push(expanded.moves);
    for (const b of room.boxes) globalBoxes.delete(b);
    // The room's boxes end on its goals.
    for (const gl of room.goals) globalBoxes.add(gl);
    player = expanded.endPlayer;
  }
  return out.join('');
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
