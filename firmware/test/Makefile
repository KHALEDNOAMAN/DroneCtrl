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

SUITES := test_control test_estimation test_safety test_telemetry
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

# The headers the board compiles have to stay C++11: that is what the AVR
# toolchain builds with. The suites themselves use C++17, so they cannot catch
# a C++17 feature creeping into a flight header; this does, in under a second.
BOARD_HEADERS := flight_loop.h attitude_ekf.h pid_controller.h mixer_math.h \
                 flight_state.h mavlink_lite.h mavlink_telemetry.h loop_timing.h \
                 rc_pulse.h hil_link.h can_telemetry.h config.h

.PHONY: cxx11
cxx11:
	@for h in $(BOARD_HEADERS); do \
		echo '#include "'$$h'"' | $(CXX) -std=c++11 -Wall -Wextra -Wpedantic -Werror \
			-fsyntax-only -I../include -x c++ - || { echo "$$h is not C++11"; exit 1; }; \
	done
	@echo "board headers are C++11 clean"

run: $(BINS) cxx11
	@fail=0; \
	for b in $(BINS); do \
		./$$b || fail=1; \
	done; \
	if [ $$fail -ne 0 ]; then echo "host tests FAILED"; exit 1; fi; \
	echo "host tests passed"

clean:
	@rm -rf $(BUILD)
