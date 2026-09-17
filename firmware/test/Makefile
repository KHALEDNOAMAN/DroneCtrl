# Host tests for the flight controller's control, estimation and safety logic.
#
# Plain g++ on purpose. These headers are deliberately free of Arduino.h, so
# the exact code that runs on the board compiles here too, and the suite needs
# nothing installed beyond a C++ compiler. That matters more than it sounds:
# a test that needs a package registry to be reachable stops running the first
# time that registry is down, and a safety test nobody can run is not one.
#
#   make        build and run everything
#   make run    same
#   make clean  remove the build directory
#
# Exit status is non-zero if any suite fails, so CI needs no extra parsing.

CXX      ?= g++
# -MMD -MP emit a .d file listing every header each suite actually included.
# Without it a change to attitude_ekf.h or flight_state.h leaves the stale
# binary in place and the suite silently reports on code that is no longer
# there, which is worse than not running it at all.
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic -I../include -MMD -MP
BUILD    := build

SUITES := test_control test_estimation test_safety
BINS   := $(addprefix $(BUILD)/,$(SUITES))
DEPS   := $(BINS:=.d)

.PHONY: all run clean
all: run

$(BUILD):
	@mkdir -p $(BUILD)

# One rule per suite. A pattern rule cannot express <name>/<name>.cpp, because
# the stem is only substituted once per prerequisite.
define SUITE_RULE
$(BUILD)/$(1): $(1)/$(1).cpp test_harness.h | $(BUILD)
	$$(CXX) $$(CXXFLAGS) -MF $$@.d $$< -o $$@
endef
$(foreach s,$(SUITES),$(eval $(call SUITE_RULE,$(s))))

-include $(DEPS)

run: $(BINS)
	@fail=0; \
	for b in $(BINS); do \
		./$$b || fail=1; \
	done; \
	if [ $$fail -ne 0 ]; then echo "host tests FAILED"; exit 1; fi; \
	echo "host tests passed"

clean:
	@rm -rf $(BUILD)
