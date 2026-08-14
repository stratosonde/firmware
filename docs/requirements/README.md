# Stratosonde Requirements & Verification Artifacts

This directory holds the living V-model / delivery artifacts produced by the
2026-08-12 intent interviews. They are **not** DDRs — normative product intent
lives in `../decisions/`. These artifacts trace requirements to those DDRs and
to implementation/proof.

| File | Role |
|---|---|
| `stratosonde-engineering-requirements-specification-first-flight.md` | Engineering requirements specification (SYS-*) for first flight |
| `requirements-traceability-matrix.md` | Merged round-2/round-3 traceability matrix (living artifact) |
| `flight1-validation-readiness-checklist.md` | Go/no-go evidence checklist for the first-flight build (DDR-0026 INV-VER-011) |
| `firmware-conformance-worklist.md` | Implementation/proof queue (FW-CONF-*) converting intent into code/test work |

Related artifacts in `../decisions/`: `system-operational-assumptions.md`
(environmental/operational assumptions the requirements are designed around)
and `open-intent-questions.md` (consolidated unresolved intent).

The objective is the closed chain:

**intent → requirement → implementation → test → evidence → flight release**
