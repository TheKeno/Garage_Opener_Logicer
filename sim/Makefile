# Simulator and tests for Garage_Opener_Logicer_R6. No board needed.
#
#   make              build everything into build/
#   make test         run the test suite
#   make run          the interactive garage simulator
#   make avr          check the sketch still compiles for an Uno (arduino-cli)
#   make clean
#
# The sketch needs the Button library; it is looked for in $(BUTTON_DIR),
# ~/Arduino/libraries/Button and the old backup drive.

R6       := ../Garage_Opener_Logicer_R6
CXX      ?= g++
# -m32 makes long 32 bits wide as on the AVR, so millis() arithmetic wraps the
# way it does on the board. Needs the 32-bit libstdc++ (lib32-gcc-libs,
# g++-multilib). ARCH= builds 64-bit, where the rollover tests are meaningless.
ARCH     ?= -m32
CXXFLAGS ?= -std=c++17 -O2 -g -Wall -Wextra
CXXFLAGS += $(ARCH)
CPPFLAGS := -I. -Istubs -I$(R6) -Ibuild/Button
STUBS    := $(wildcard stubs/*.h)

FIRMWARE_SRC := firmware.cpp $(R6)/DistanceSensor.cpp $(R6)/LightPulseSensor.cpp \
                $(R6)/ParkAssist.cpp $(R6)/AccelDial.cpp build/Button/Button.cpp
FIRMWARE_OBJ := $(patsubst %.cpp,build/fw/%.o,$(notdir $(FIRMWARE_SRC)))
TEST_OBJ     := $(patsubst tests/%.cpp,build/%.o,$(wildcard tests/*.cpp))

all: build/firmware.so build/garage_sim build/run_tests

build/Button/Button.cpp:
	@mkdir -p build
	@for d in "$(BUTTON_DIR)" "$$HOME/Arduino/libraries/Button" \
	          "/mnt/old/backup from main drive/documents/Arduino/libraries/Button"; do \
		if [ -n "$$d" ] && [ -f "$$d/Button.cpp" ]; then ln -sfn "$$d" build/Button; exit 0; fi; \
	done; \
	echo "Cannot find the Button library. Run: make BUTTON_DIR=/path/to/libraries/Button" >&2; exit 1

# The firmware: position-independent, and with no GNU-unique symbols, which
# would stop dlclose() from unloading it and leave stale globals behind.
build/fw/%.o: $(R6)/%.cpp $(STUBS) | build/Button/Button.cpp
	@mkdir -p build/fw
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -fPIC -fno-gnu-unique -c $< -o $@
build/fw/firmware.o: firmware.cpp $(R6)/*.ino $(R6)/*.h $(STUBS) | build/Button/Button.cpp
	@mkdir -p build/fw
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -fPIC -fno-gnu-unique -c $< -o $@
build/fw/Button.o: build/Button/Button.cpp $(STUBS)
	@mkdir -p build/fw
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -fPIC -fno-gnu-unique -Wno-reorder -c "$<" -o $@
build/firmware.so: $(FIRMWARE_OBJ)
	$(CXX) $(ARCH) -shared $^ -o $@

# The host: the simulated garage, exporting the Arduino core to the firmware.
build/%.o: %.cpp sim.h $(STUBS) | build/Button/Button.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@
build/%.o: tests/%.cpp tests/check.h sim.h $(STUBS) | build/Button/Button.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@
build/garage_sim: build/garage_sim.o build/sim.o
	$(CXX) $(ARCH) -rdynamic $^ -ldl -o $@
build/run_tests: $(TEST_OBJ) build/sim.o
	$(CXX) $(ARCH) -rdynamic $^ -ldl -o $@

test: all
	./build/run_tests

run: all
	./build/garage_sim

avr: build/Button/Button.cpp
	arduino-cli compile -b arduino:avr:uno --library "$$(readlink -f build/Button)" $(R6)

clean:
	rm -rf build

.PHONY: all test run avr clean
