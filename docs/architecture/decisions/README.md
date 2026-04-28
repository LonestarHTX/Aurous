# Architecture Decision Records

This directory contains accepted and proposed architectural decisions for the
active Aurous tectonics path.

## Decisions

| ADR | Status | Summary |
| --- | --- | --- |
| `0001-c-freeze-voronoi-ownership-decoupled-material.md` | Accepted | Freezes Prototype C as the ownership/projection foundation: nearest-center Voronoi ownership, raw adjacency boundaries, and decoupled material projection. |
| `0002-d-persistent-divergent-ocean-crust.md` | Accepted | Defines Prototype D's first persistent crust slice: sidecar-owned divergent ocean crust state, event log, authority hashing, and strict Slice 1 scope before event code lands. |
| `0003-d-oceanic-crust-cooling-law.md` | Accepted | Defines Slice 6's age-derived ocean-crust elevation cooling law while preserving D crust identity and C ownership/boundary invariants. |

## Template

Use this shape for future decisions:

- status: proposed, accepted, superseded
- context
- decision
- invariants
- consequences
- alternatives considered
- validation/evidence
- follow-up work
