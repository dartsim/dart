# Island deactivation (sleeping)

Visual demo of DART's automatic body deactivation and the solver-island
partition.

Several separated box stacks each form an independent **solver island**. Each
island is drawn in its own color. When an island settles and goes to sleep
(deactivates), it is tinted to a dim, cool shade — at that point its constraint
solve, forward dynamics (gravity), and position integration are all skipped each
step until something disturbs it.

## Controls

| Key | Action |
|-----|--------|
| `f` | Shoot a sphere horizontally at the next stack — it flies in and wakes the sleeping island on impact |
| `d` | Drop a box from above onto the next stack — it wakes the island it lands on, and merges two islands into one (same color) if it bridges them |
| `t` | Toggle automatic deactivation on/off (with it off, nothing ever dims) |
| `s` | Print the number of awake vs. asleep skeletons |

A yellow arc shows the predicted parabolic path of the next sphere you fire, so
you can see roughly where it will land (it is only an estimate — gravity and the
collision itself make the real impact differ).

Projectiles you fire/drop stay bright orange so you can track them; the stacks
take island colors and dim when asleep.

## What to watch

- Each stack starts in its own bright color and, after settling, fades to the
  dim "asleep" tint.
- Toggling the feature off (`t`) keeps every island bright — that is the
  baseline cost (every island solved every step). Toggling it back on lets them
  sleep again.
- Dropping a box (`d`) onto a sleeping stack re-brightens it (wake), and if the
  box bridges two stacks they take on a single color (the islands merged).
