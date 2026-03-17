# M1-M5 Lessons Learned

**Date:** March 17, 2026
**Status:** Project cleanup checkpoint after M5d

## Snapshot

At this checkpoint:
- M1-M5 are functionally implemented at the 60k correctness tier
- the preferred periodic maintenance architecture is `PreserveOwnershipPeriodic`
- preserve-mode M4 collision execution works through cached boundary-contact evidence
- deterministic and paper-style rifting paths both exist

This is a good place to pause large architectural changes, clean the project surface, and resume from a more stable base.

## What We Learned

### 1. Ownership churn was the real boundary problem

The original boundary degradation was not mainly a visualization issue and not mainly a subduction/collision issue.

The core problem was repeated periodic ownership rewriting:
- one resample already damaged clean initial boundaries
- repeated periodic full resampling accumulated that damage quickly

The key lesson:
- periodic maintenance and ownership/topology change must be separated

### 2. `PreserveOwnershipPeriodic` is the right maintenance architecture

The first architecture that simultaneously gave:
- clean boundaries
- healthy gaps/overlaps
- continued world evolution

was `PreserveOwnershipPeriodic`:
- same-plate exact hit first
- same-plate nearest-triangle recovery second
- full ownership query only as fallback

Lesson:
- periodic maintenance should preserve ownership by default
- explicit topology changes should happen only on events

### 3. Clean boundaries require clean event plumbing too

Once ownership was cleaned up, old overlap-heavy event detection stopped working.

That led to the preserve-mode M4 path:
- boundary-contact detection
- persistence across preserve resamples
- cached boundary-contact collision execution in same-step `CollisionFollowup`

Lesson:
- once the simulation stops producing noisy overlap fields, events must execute from explicit cached evidence instead of trying to rediscover themselves from overlap noise

### 4. Deterministic infrastructure slices beat one-shot milestone prompts

M5 worked because it was split into clear layers:
- M5a: plate lifecycle + forced 2-way rift
- M5b: terrane inheritance measurement + post-rift coherence
- M5c: deterministic generalization to 2-4 children
- M5d: paper-style automatic trigger + warped fracture boundaries

Lesson:
- for structurally risky systems, deterministic slices are much safer than asking for the whole paper feature in one shot

### 5. Plate lifecycle had to come before “paper-like” rifting

Before stochastic rifting could work, the runtime needed:
- stable plate ids
- child plate creation
- parent retirement
- same-step `RiftFollowup`

Lesson:
- lifecycle/integration work is the real prerequisite; the fracture geometry itself is the easy part

### 6. Performance understanding is already good enough for roadmap decisions

Current performance lesson:
- 60k is viable for correctness and architecture iteration
- 500k is still dominated by the subduction distance field and is nowhere near a production budget yet

Lesson:
- defer serious optimization until the tectonic cycle is in place
- but keep perf visibility on at all times

## Current Caveats

### M4 caveat

M4 is functionally in, but the current preserve-mode collision result is not yet fully satisfying visually/plausibly.

The architecture is good:
- cached contact collision works
- boundaries stay clean through collision

But collision shaping/tuning may still need another focused pass later.

### M5 caveats

M5 is functionally in, but two issues are still visible:
- automatic rifting currently fires very early in the 60k harness and is likely too eager
- the medium 2-way case improved under warped boundaries, but sibling-boundary closure by step 50 is not fully solved

These look like tuning problems, not missing infrastructure.

### Performance caveat

500k performance is still not acceptable:
- current production-scale blocker remains subduction distance propagation

## Recommended Resume Point

If resuming from this checkpoint, the safest order is:

1. Do project cleanup / consolidation
2. Run a broader non-500k regression pass on the chosen architecture
3. Start M6 as a tuning phase, not an architecture phase

Suggested early M6 topics:
- M4 collision plausibility tuning
- automatic rift trigger rate / cooldown tuning
- child motion persistence, especially smaller 2-way rifts
- longer-run tectonic-cycle behavior

Only after the cycle feels stable should performance become the primary focus.

## Things Not To Reopen Lightly

These decisions now have enough evidence behind them that reopening them should require a concrete failure:

- `PreserveOwnershipPeriodic` as the maintenance architecture
- cached boundary-contact collision execution
- stable plate id / explicit plate lifecycle
- deterministic-first implementation strategy for major tectonic systems
