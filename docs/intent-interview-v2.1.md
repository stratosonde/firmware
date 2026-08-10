---
name: intent-interview
description: >
  A development method for extracting durable product design from a developer's
  mental model, expressing that design as implementation-independent intent,
  behavioral requirements, and design/decision records, then proving that an
  implementation conforms to it. Use both retroactively (to reconstruct and
  validate an existing, often LLM-written codebase) and proactively (to design a
  system before code exists). Also use for restructuring ADRs/DDRs into live
  contracts, identifying intent-vs-documentation-vs-code drift, or running an
  audit/bug pipeline that must not introduce new bugs. Trigger whenever the work
  involves "does the code actually do what I intend," "could we rebuild this from
  the design alone," requirement elicitation, architectural decision capture,
  mental-model validation, or reviewing LLM-generated code the developer did not
  author line-by-line.
---

# The Intent Interview

*(a.k.a. Intent-Anchored Development — extract the design, then prove the code)*

## Why this method exists

Traditionally, a developer earns a deep mental model of a system by **writing** it.
By the time the code compiles, they understand it, and that understanding is the
real deliverable — the code is just its residue.

When an LLM writes the code, that residue is gone. The implementation can grow much
faster than the developer's mental model. The code can even be locally correct while
the developer's understanding of the product silently drifts out of sync with it.
For systems that cannot easily be recalled or patched after deployment — embedded
firmware, spacecraft, a balloon in the stratosphere, a shipped medical device — the
gap between intended behavior and actual behavior is a major risk surface.

But merely validating today's code is not enough.

The more durable goal is to **extract the product design itself**: the intent,
invariants, behavioral requirements, trade-offs, and decisions that should remain
true even if the current implementation is deleted and rebuilt from scratch.

The method therefore has two jobs:

1. **Extract the design from the developer's mind.** Turn tacit engineering judgment
   into a coherent, implementation-independent body of product intent and DDRs.
2. **Validate implementations against that design.** Treat the current code as one
   implementation that must prove conformance to the extracted design — not as the
   source of truth for what the product was supposed to be.

A successful result is not merely "the developer now understands this repository."
A successful result is:

> **The product is specified well enough that an independent competent implementer
> — human or LLM — could rebuild it from the intent set and arrive at substantially
> the same externally meaningful behavior, even if the internal implementation is
> different.**

That is the regeneration criterion for this method.

---

## The principles everything hangs on

### 1. Intent is authoritative; code is evidence

The implementation can reveal what the system currently does. It cannot reveal why
that behavior is required, which trade-off made it desirable, or whether it is
accidental.

The product definition therefore flows in this direction:

**Developer intent → behavioral contract → design decisions → implementation → proof**

Never reverse that arrow merely because code already exists.

### 2. Nothing about implementation conformance is accepted on argument

> **Every claim that an implementation satisfies or violates an intent must
> eventually be demonstrated against the real implementation by executable proof.**

Proof may be a host test, integration test, fault-injection test, hardware-in-loop
test, protocol test, bench test, or other repeatable verification appropriate to the
system. The exact test mechanism may vary; proof by persuasive prose does not.

### 3. The interview must not be contaminated by the implementation

In retroactive mode, do **not** use the existing code or DDRs to feed the developer
answers before their intent has been captured.

Do not ask:

> "The firmware uses the last known GNSS position after a timeout. Is that correct?"

Ask:

> "GNSS has failed for the last several wake cycles. What should the product do now,
> and why?"

Record the answer first. Inspect the current design records and code second.
Otherwise the exercise degenerates into validating existing artifacts against
themselves.

---

## The role split

### The developer supplies the WHY and the engineering judgment

The developer owns:

- mission intent;
- invariants — what must always or never be true;
- priorities when objectives conflict;
- acceptable degradation;
- safety and regulatory boundaries;
- failure philosophy;
- trade-offs and rationale;
- what information or capability is more valuable when everything cannot be kept;
- which behaviors are intentional versus merely historical.

This knowledge often exists tacitly. The interviewer's job is to pull it out without
forcing the developer to translate it into code or formal requirements while
speaking.

### The interviewer/LLM supplies structure, challenge, and the WHAT

The LLM owns:

- interviewing one bounded topic at a time;
- separating WHY from HOW;
- probing edge cases, failure cases, and conflicting priorities;
- turning conversational intent into precise behavioral contracts and DDRs;
- reading the implementation after intent has been captured;
- determining what the implementation actually does;
- mapping existing design records to the newly elicited intent;
- computing the diff between **intent, documented design, and implementation**;
- constructing executable proof of conformance or divergence.

The LLM may infer WHAT from code. It must never fabricate WHY.

If the rationale is unknown, that is an interview question or an explicitly recorded
open decision — never a blank for the LLM to fill with a plausible story.

---

# The design artifact stack

The Intent Interview produces several layers. They have different authority and
should not be collapsed into one document.

## 1. Interview record — provenance, not specification

The conversation is raw evidence of what the developer said, including uncertainty,
contradictions, tentative ideas, and later corrections.

Keep it for provenance, but do not treat an unedited transcript as normative design.

## 2. Product intent / invariants — the enduring WHY

These are implementation-independent truths such as:

- the mission should continue through recoverable subsystem failures;
- measurements must never silently masquerade as fresher or more accurate than they
  are;
- energy expenditure must not jeopardize the ability to continue the mission;
- data that cannot currently be transmitted should remain recoverable when feasible.

These should survive a complete rewrite of the software.

## 3. Behavioral requirements — the observable contract

Translate intent into explicit, testable statements about required behavior. Prefer
normative language such as **SHALL**, **SHALL NOT**, and **MAY** where useful.

Behavioral requirements answer:

> "What would another implementer need to make true in order to satisfy this
> intent?"

They should specify externally meaningful behavior and edge-case policy without
unnecessarily prescribing source structure, function names, or algorithms.

Example:

> If a fresh position cannot be obtained within the allowed acquisition budget, the
> system SHALL terminate acquisition, retain the previous valid position as stale,
> identify the position as stale in downstream records, and continue the mission.

That is stronger than either a prose rationale or an implementation description.

## 4. DDRs — chosen engineering decisions and trade-offs

A Design/Decision Record captures **how the product has chosen to satisfy one or more
intents**, together with why that decision was selected over plausible alternatives.

A DDR may contain algorithmic or architectural policy when that policy is itself a
meaningful design choice — for example newest-first archive recovery, a one-way
mission state transition, or retaining the previous regulatory region after a stale
position.

DDRs are not source-code documentation. They should remain meaningful across a
rewrite.

## 5. Implementation bindings — today's realization

Only after the design is captured do we bind it to the current implementation:

- modules;
- functions;
- state machines;
- stored fields;
- wire formats;
- hardware interfaces;
- configuration;
- generated artifacts.

Bindings are allowed to change when the implementation changes without changing the
underlying product intent.

## 6. Proof — executable conformance

Each important behavioral contract and DDR should have one or more proofs showing
that the implementation actually satisfies it.

The proof should fail when the underlying intent is materially violated, not merely
when a particular implementation detail changes.

---

# The canonical Intent Contract

For each bounded feature or decision, produce the following structure.

## INTENT

Why does this behavior exist? What mission-level property are we protecting?

## INVARIANTS

What must always be true? What must never happen?

## BEHAVIORAL REQUIREMENTS

What shall the product observably do in the nominal case, degraded case, timeout
case, restart case, and boundary cases relevant to this intent?

## DECISION

What architectural or behavioral policy have we chosen to satisfy the intent?

## RATIONALE / TRADE-OFFS

Why this decision? What competing objectives were consciously traded away?

## OPEN DECISIONS

What has not actually been decided yet? Do not convert uncertainty into a fake
requirement.

---

# Implementation binding

When an implementation exists, append:

## DELIVERABLE

Which concrete implementation elements currently realize the contract?

## PROOF

Which executable tests or repeatable verification procedures demonstrate that the
implementation honors the contract?

This creates a clean separation:

**Design contract:** Intent → Invariants → Behavioral Requirements → Decision → Rationale

**Implementation binding:** Deliverable → Proof

The first half can exist before a single line of code is written. The second half
proves a particular implementation.

---

# Workflow A — Design Extraction Interview

Use this whether or not code already exists.

The objective is not to interrogate source code. The objective is to extract a
complete enough model of the product that the design can stand independently of its
current implementation.

## 1. Pick one bounded topic

Avoid attempting "the whole firmware" at once.

Choose a feature, transition, resource, subsystem interaction, failure condition, or
policy small enough to reason about clearly. Examples:

- one wake cycle;
- GNSS acquisition;
- battery/energy gating;
- stale sensor handling;
- regulatory-region selection;
- local science-data persistence;
- telemetry/link probing;
- archive recovery;
- reset/watchdog behavior;
- peripheral ownership;
- commissioning-to-flight transition.

## 2. Start with an ordinary scenario

Ask the developer to narrate what they believe should happen in plain language.
Do not ask about function names, variables, or existing code structure.

## 3. Pull for WHY

Whenever the answer becomes implementation-shaped, ask why the behavior is needed.

A *how* is:

> "Disable GPS below voltage X."

A *why* is:

> "A high-current acquisition must never cause a brownout that sacrifices the rest
> of the mission."

Capture the WHY first. The threshold may be a later decision or parameter.

## 4. Probe the boundaries

Do not stop after the happy path. Ask scenarios that expose engineering judgment:

- What if the operation times out?
- What if the subsystem disappears permanently?
- What if power is insufficient?
- What if data is stale?
- What if storage is almost full?
- What if communications are unavailable for hours or days?
- What if only one of two valuable actions can be afforded?
- Which data wins when all data cannot be transmitted?
- What must happen across reset or brownout?
- What is recoverable versus mission-ending?
- What behavior would be actively dangerous or unacceptable?

## 5. Force priority decisions where objectives conflict

The most valuable design knowledge often appears only when two good outcomes cannot
both be achieved.

Examples:

- newest data versus complete historical recovery;
- radio transmission versus energy preservation;
- fresh positioning versus continuing the wake cycle;
- perfect delivery guarantees versus airtime and battery;
- strict subsystem correctness versus forward mission progress.

Do not invent the answer. Let the developer expose the product's value hierarchy.

## 6. Preserve uncertainty

"I haven't decided yet" is a valid and useful result.

Record it as an **OPEN DECISION**. Do not let the interviewer silently convert the
current implementation into the answer merely because code already picked something.

## 7. Synthesize, then confirm

Turn the conversation into a concise Intent Contract. Read back the substantive
intent, invariants, requirements, decisions, trade-offs, and open questions.

The developer should correct the design artifact — not source code — until it
accurately represents the intended product.

## 8. Add or update DDRs

Once an intent is stable, create the DDR or DDRs needed to preserve the meaningful
engineering choices behind it.

A single interview topic may produce:

- one overarching invariant;
- several behavioral requirements;
- zero, one, or several DDRs;
- one or more explicit open decisions.

Do not force a one-interview-to-one-DDR mapping.

---

# Workflow B — Retroactive Three-Way Validation

Use this when code and/or existing DDRs already exist.

The critical ordering is:

> **Interview first. Freeze the elicited intent. Then inspect documents and code.**

For each captured contract compare three independent artifacts:

**DEVELOPER'S CURRENT INTENT ↔ EXISTING DESIGN RECORDS ↔ CURRENT IMPLEMENTATION**

## Outcomes

### MATCH

Intent, design record, and code materially agree.

Action: bind the intent to its deliverable and executable proof. Move on.

### CODE DRIFT

The developer's confirmed intent and design record agree, but the implementation
differs.

Action: treat as a candidate defect. Prove the divergence with a failing test before
changing code.

### DOCUMENT DRIFT

The developer's confirmed intent and implementation agree, but the DDR/specification
differs or is incomplete.

Action: update the design artifact. Do not change correct code merely to preserve a
stale document.

### MODEL DRIFT

The developer's current mental model differs from both the existing design record and
implementation.

This does **not** automatically mean the developer is wrong. Stop and determine
whether:

- the original decision is still desired and the developer had forgotten it;
- the desired product has changed and the old design should be superseded;
- the existing documents and code both fossilized an accidental behavior.

The resolution is itself design work and should be recorded explicitly.

### SPLIT DRIFT

Intent, documentation, and code disagree in different ways.

Action: do not patch immediately. Reconstruct the decision history and resolve which
behavior should become authoritative before changing implementation.

### SILENCE

The implementation or existing DDR clearly contains behavior the developer never
mentioned and for which no elicited intent exists.

Surface it as a question:

> "The current system does X. What product intent, if any, should justify that?"

The result may become a newly discovered requirement, an obsolete historical choice,
or accidental code to remove.

### OPEN DECISION

The interview deliberately left the behavior undecided.

If current code already implements one answer, record it as **current behavior, not
approved intent**, until the developer makes the product decision.

---

# Workflow C — Proactive Design / Greenfield Generation

Use the same method before code exists.

1. Interview bounded topics until the important product behaviors and cross-cutting
   policies are represented by Intent Contracts.
2. Create the DDR set that records meaningful architectural and behavioral choices.
3. Build a coverage matrix showing which major product responsibilities have been
   interviewed and which remain open.
4. Derive executable acceptance tests from the behavioral requirements before or
   alongside implementation.
5. Give the resulting intent set, DDRs, interfaces, constraints, and proof targets to
   the implementer or coding LLM.
6. Generate the implementation.
7. Run the proof suite and correct implementation differences without silently
   changing the design.

This is the proactive form of the same technique.

The specification is upstream of code, but the conversation that created it can still
be informal and human. The method's job is to turn that conversation into durable
engineering artifacts.

---

# The regeneration test

Periodically ask:

> **If the repository disappeared today, could a competent implementation team build
> a behaviorally equivalent product from the Product Intent Set and DDRs without
> having to guess important mission behavior?**

If the answer is no, identify what they would have to guess. Those guesses define the
next interview topics.

Behaviorally equivalent does **not** mean byte-for-byte or function-for-function
identical. Internal architecture may differ unless a particular architecture is
itself a recorded decision.

The target is equivalence in:

- mission behavior;
- externally observable interfaces;
- invariants;
- failure and degradation behavior;
- data semantics;
- persistence semantics;
- state transitions;
- resource priorities;
- safety/regulatory behavior;
- intentional trade-offs.

---

# Building a complete product intent set

A collection of interesting DDRs is not automatically a complete product design.
Track coverage deliberately.

For embedded/mission firmware, interview across at least these dimensions when
applicable:

## Mission lifecycle

- boot;
- commissioning;
- arming/deployment;
- nominal mission cycle;
- state transitions;
- recovery after reset/brownout;
- end-of-mission behavior.

## Resources

- energy/power budget;
- compute/time budget;
- storage;
- airtime/bandwidth;
- bus ownership and peripheral contention.

## Data lifecycle

- acquisition;
- validity/freshness;
- timestamping;
- persistence;
- integrity;
- compression/packing;
- transmission;
- acknowledgement;
- retransmission/recovery;
- deletion/retirement.

## Communications

- network acquisition;
- region/regulatory behavior;
- link adaptation;
- disconnected operation;
- backhaul recovery;
- downlink/control behavior;
- sequence/deduplication semantics.

## Failure philosophy

- timeout policy;
- retry policy;
- graceful degradation;
- stale-state handling;
- watchdog/reset policy;
- permanently failed subsystem behavior;
- what, if anything, is truly mission-ending.

## Cross-cutting invariants

- forward progress;
- data honesty;
- mission ownership;
- safety;
- regulatory compliance;
- persistence guarantees;
- monotonic or one-way state;
- resource priority.

The specific categories will differ by product. Their purpose is to expose blind
spots, not to force every project into the same architecture.

---

# Traceability

Maintain a simple trace from design to implementation:

**Intent ID → Requirement(s) → DDR(s) → Deliverable(s) → Proof(s)**

Also permit the reverse query:

**Code behavior → Which intent authorizes this?**

If significant behavior has no answer, that is SILENCE and deserves review.

A useful project-level output is a coverage matrix containing:

- topic / feature;
- intent status;
- requirement IDs;
- DDR IDs;
- open decisions;
- implementation binding status;
- proof status;
- drift status.

This allows the team to measure design completeness independently of line coverage.

---

# Characterization mode

When the developer genuinely does not know what the product should do yet, do not
force a fake belief.

If code exists, characterize current behavior with a test and read the result back to
the developer as evidence:

> "This is what today's implementation does. Do you want that behavior to become an
> intentional product decision?"

Possible results:

- adopt it as intent;
- reject it and define the desired behavior;
- leave it temporarily open;
- identify it as obsolete or accidental behavior.

Characterization tests describe current reality. They become normative proof only
after the underlying behavior is intentionally accepted.

---

# Workflow D — Proof-Before-Fix (audit / bug pipeline)

Use this to audit a repository and fix findings **without turning reviewer opinions
into code churn**.

## The pipeline

1. **Generate candidates.** An audit pass proposes possible findings.
2. **Trace to intent.** Determine which requirement/invariant the candidate would
   violate. If no intent exists, first decide whether the behavior is actually
   undesired rather than automatically calling it a bug.
3. **Prove, don't argue.** Before a candidate becomes an issue, write a failing test
   or equivalent repeatable proof that demonstrates the divergence against the real
   implementation.
4. **Drop unproven findings.** A claim that cannot demonstrate a violation is not an
   accepted bug merely because its explanation sounds convincing.
5. **Fix one proven issue at a time.** One finding → one issue → one fix → one commit
   where practical.
6. **Turn the proof green.** The reproducing test becomes a permanent guard against
   regression.

The proof belongs at issue-creation time, not fix time. This prevents false findings
from ever reaching the modification stage.

## Reading the trend

Repeated audits should show diminishing severity if the system is converging. That is
a healthy signal, not a reason to relax the proof discipline. The discipline is what
prevents each new round from reintroducing old classes of failure.

---

# DDR shape produced by this method

A DDR should normally contain at least:

## Context

What problem, constraint, or conflicting objective required a decision?

## Intent

Which higher-level product intent or invariant is being protected?

## Behavioral contract

What observable requirements must any conforming implementation satisfy?

## Decision

What policy or architecture has been selected?

## Rationale

Why was this selected? What alternatives or trade-offs matter?

## Consequences

What becomes easier, harder, impossible, or intentionally sacrificed?

## Open questions

What remains undecided or intentionally deferred?

## Implementation binding

Where does the current implementation realize the decision? Keep this section easy to
update as code moves.

## Proof

Which tests demonstrate conformance?

A DDR written this way is simultaneously historical rationale, current design
contract, and traceability anchor — without confusing the current source tree with
the product definition.

---

# Guardrails / anti-patterns

- **Do not let existing code answer the interview.** Capture the developer's intent
  before revealing what today's implementation does.
- **Do not let existing DDRs prime the developer.** Existing records are a third
  artifact to compare after the interview, not an answer key.
- **Keep pulling for WHY.** If the developer gives a mechanism, ask what invariant or
  trade-off makes that mechanism desirable.
- **Do not mistake a parameter for an intent.** Thresholds, retry counts, timeouts,
  packet sizes, and specific algorithms may be decisions or configuration, not the
  underlying requirement.
- **Do not manufacture completeness.** Unknown behavior should remain OPEN DECISION
  until consciously resolved.
- **Do not make implementation details normative accidentally.** Function names and
  module boundaries belong in implementation bindings unless the architecture itself
  is a deliberate product decision.
- **Do not treat the developer's first recollection as infallible.** MODEL DRIFT is a
  legitimate outcome. Resolve it explicitly rather than silently rewriting history.
- **Do not call surprising code a bug without intent.** First identify the violated
  contract; then prove the violation.
- **Do not accept conformance or bugs on argument alone.** Demand executable or
  otherwise repeatable proof appropriate to the system.
- **Do not re-read the whole codebase to the developer.** Surface differences,
  silences, and decisions that require human judgment.
- **Do not invent rationale.** The LLM owns structure and implementation analysis;
  the developer owns engineering intent.
- **Prefer scenarios over leading questions.** "What should happen if GNSS is gone
  for an hour?" is better than "Should we use the last known position?"
- **Separate current behavior from desired behavior.** Characterization is evidence,
  not automatic specification.

---

# Suggested session rhythm

A productive interview session should feel conversational rather than bureaucratic.

1. Interview one small feature.
2. Ask enough failure/priority questions to expose the real WHY.
3. Summarize the proposed Intent Contract aloud.
4. Resolve corrections or open decisions.
5. Freeze that intent for the validation pass.
6. If code exists, inspect DDRs and implementation privately.
7. Return only meaningful MATCH / DRIFT / SILENCE results.
8. Create or update the DDR and requirements.
9. Add implementation bindings and proofs.
10. Move to the next uncovered product behavior.

Do not interrupt the interview after every sentence to audit code. Preserve the human
reasoning flow first; validate in bounded passes second.

---

# Expected project outputs

Over time, this method should produce:

1. **Interview records** — provenance of the developer's reasoning.
2. **Product Intent Catalog** — the implementation-independent mission intent,
   invariants, and behavioral requirements.
3. **DDR set** — the durable engineering decisions and trade-offs that shape the
   product.
4. **Open Decision Register** — questions that have deliberately not yet been
   answered.
5. **Traceability matrix** — intent → requirements → DDRs → implementation → proof.
6. **Proof suite** — executable evidence that the current implementation conforms.
7. **Drift report** — mismatches among current intent, design records, and code.

The Product Intent Catalog and DDRs are the artifacts from which a future
implementation should be derivable. The implementation bindings and proof suite show
that today's repository is one conforming realization of them.

---

# Drop-in quickstart prompt — retroactive project

Paste this into a fresh LLM session that has access to the repository:

> We're going to use the **Intent Interview / Intent-Anchored Development** method.
>
> The primary goal is not to explain or document the current code. The goal is to
> extract the enduring product design from my mental model so completely that, in
> principle, the current implementation could be deleted and a competent human or
> LLM could rebuild a behaviorally equivalent product from the resulting Product
> Intent Catalog and DDRs.
>
> Treat my intent as upstream of the implementation. The existing DDRs and code are
> evidence to compare later, not an answer key for the interview.
>
> Interview me **one small bounded feature at a time**. Ask in plain language what I
> expect the product to do and, more importantly, why. Use scenario questions to
> expose invariants, failure behavior, degraded behavior, priorities, trade-offs,
> and what must never happen. If I describe a HOW, pull me toward the WHY. If I say
> I haven't decided something, record an OPEN DECISION; do not infer the answer from
> existing code.
>
> Do not show me or paraphrase the relevant code or existing DDR before my intent for
> that topic is captured. Avoid leading questions that reveal the current
> implementation.
>
> After enough interviewing to understand a bounded topic, synthesize a draft Intent
> Contract containing: INTENT, INVARIANTS, BEHAVIORAL REQUIREMENTS, DECISION,
> RATIONALE / TRADE-OFFS, and OPEN DECISIONS. Let me correct that design before you
> validate it.
>
> Once the intent is frozen, inspect the existing DDRs and real implementation and
> compare three things independently: MY CURRENT INTENT, THE EXISTING DESIGN RECORDS,
> and THE CURRENT CODE.
>
> Report only meaningful outcomes: MATCH, CODE DRIFT, DOCUMENT DRIFT, MODEL DRIFT,
> SPLIT DRIFT, SILENCE, or OPEN DECISION. Do not assume my current recollection is
> automatically right; if the old DDR and code agree with each other but disagree
> with me, surface the conflict and interview me until we decide whether the product
> intent changed or my model drifted.
>
> For each resolved topic, create or update the appropriate DDR(s) and behavioral
> requirements. Keep the design implementation-independent. Then add an
> implementation binding identifying today's modules/functions and a PROOF section
> identifying the test(s) that demonstrate conformance.
>
> Never accept a bug or a claim of conformance merely on argument. A candidate code
> defect must be tied to a violated intent/requirement and reproduced against the
> real implementation with a failing test or other repeatable executable proof
> before it becomes an accepted finding.
>
> Maintain coverage across the product rather than only documenting interesting
> discoveries. Track uncovered features, failure policies, interfaces, lifecycle
> states, data semantics, resource priorities, and open decisions. Periodically ask:
> "If this repository vanished today, what important behavior would a new
> implementer still have to guess?" Use those guesses to select the next interview
> topics.
>
> Start the interview yourself. Pick one small, high-leverage product behavior and
> ask me the first scenario question. Do not ask me to choose a subsystem unless a
> choice is genuinely necessary.

---

# Drop-in quickstart prompt — greenfield project

> We're going to design this product using the **Intent Interview / Intent-Anchored
> Development** method before implementation.
>
> Interview me one bounded behavior at a time and extract the product's WHY,
> invariants, behavioral requirements, priorities, failure philosophy, and deliberate
> trade-offs. Challenge the design with scenarios and edge cases rather than asking
> me to specify source-code structure.
>
> For each resolved topic, produce an implementation-independent Intent Contract and
> the DDR(s) required to preserve important engineering decisions. Keep undecided
> behavior explicitly OPEN rather than making assumptions.
>
> Maintain a coverage matrix so the resulting intent set describes the complete
> product rather than only the topics that happened to come up first.
>
> The target is a design package complete enough that a competent human or LLM can
> implement the system without guessing important mission behavior. From the
> behavioral requirements, derive executable acceptance tests. Then use those tests
> to prove that the eventual implementation conforms to the design.
>
> Start by selecting one small, high-leverage behavior and interviewing me about it.


## Legacy Design Migration

When applying this method to a project that already has older ADRs, DDRs, requirements, or design notes, those artifacts are **migration input**, not dependencies of the new design set.

The migration rule is:

1. **Interview first.** Elicit the developer's intent without using the legacy record as the answer key.
2. **Mine legacy material second.** After intent has been captured, inspect old records for useful rationale, invariants, constraints, trade-offs, requirements, and historical decisions.
3. **Reconcile.** Compare legacy material with the newly elicited intent and with the implementation.
4. **Absorb what survives.** Every still-valid piece of product design knowledge must be incorporated into the new V2 product-intent/DDR set.
5. **Do not create permanent references back to legacy DDRs.** A new V2 DDR must stand alone and remain understandable if the old record disappears.
6. **Retire only when lossless.** A legacy record may be marked migrated, retired, or removed only when it contains no unique design knowledge that is absent from the new design set.

The target state is a **self-contained product design corpus**. A clean-room implementer should be able to reconstruct the intended product without needing the legacy DDR archive.

A migration audit should therefore answer one question before deleting each old record:

> **If this legacy record disappeared now, would any rationale, invariant, requirement, constraint, trade-off, or historically important design fact disappear with it?**

If yes, migration is incomplete. If no, the legacy record has served its purpose and may be removed.

