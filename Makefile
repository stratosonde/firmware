# Root wrapper (pretest-hardening handoff 2026-08-15, Phase 1 / PIPE-02):
# stable entry points over the existing build systems. The underlying
# Makefiles (tests/host/Makefile, Debug/Makefile) remain authoritative;
# this only forwards, so CI and local runs share one vocabulary.

.PHONY: check characterization contracts integration structural sanitize lint \
        arm-debug arm-flight release-gate host-clean

check:            ; $(MAKE) -C tests/host check
characterization: ; $(MAKE) -C tests/host characterization
contracts:        ; $(MAKE) -C tests/host contracts
integration:      ; $(MAKE) -C tests/host integration
structural:       ; $(MAKE) -C tests/host structural
sanitize:         ; $(MAKE) -C tests/host sanitize
release-gate:     ; $(MAKE) -C tests/host release-gate
host-clean:       ; $(MAKE) -C tests/host clean

# lint: hygiene gate - cppcheck over first-party sources (gated,
# error-exitcode) + clang-format changed-lines (HARD GATE, 2-space LLVM,
# owner decision 2026-08-16) + actionlint (CI hygiene job). Requires
# bash and the tools (build box / CI ubuntu); on Windows run via Git Bash.
lint:
	bash tools/run_cppcheck.sh
	bash tools/check_format.sh

arm-debug:
	$(MAKE) -C Debug clean
	$(MAKE) -C Debug -j16 all

# arm-flight: the deterministic PROJECT_CPPFLAGS path replaces the sed
# mutation in Phase 6 (PIPE-04). Until then the flight configuration is built
# exactly as the CI firmware-flight job does it.
arm-flight:
	@echo "arm-flight: deterministic flag path lands in Phase 6; see ci.yml firmware-flight job" & exit 1
