# Software-in-the-loop harness.
#
# Links the flight controller's own headers against a rigid body model and
# flies a set of scenarios, including injected faults. Exit status is non-zero
# if any scenario misses its criteria, so CI needs no extra parsing.
#
#   make        build and fly
#   make run    same
#   make clean  remove the binary and the logs

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -I../include -MMD -MP
LOGS     := logs

.PHONY: all run monte-carlo clean
all: run

sil_flight: sil_flight.cpp
	$(CXX) $(CXXFLAGS) -MF sil_flight.d $< -o $@

$(LOGS):
	@mkdir -p $(LOGS)

run: sil_flight | $(LOGS)
	./sil_flight --out $(LOGS)

# Holds the gains fixed and randomises the airframe, to answer the question the
# rest of the repository leaves open: how far can the real aircraft be from the
# modelled one before these gains stop working.
monte-carlo: sil_flight | $(LOGS)
	./sil_flight --monte-carlo 800 --out $(LOGS)

clean:
	@rm -rf sil_flight sil_flight.d $(LOGS)

-include sil_flight.d
