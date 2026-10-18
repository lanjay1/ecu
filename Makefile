CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Iinclude -Isim -Itest/native
TESTS := $(wildcard test/native/test_*.cpp)
BINS := $(TESTS:test/native/%.cpp=build/%)

all: test

build:
	mkdir -p build

build/%: test/native/%.cpp | build
	$(CXX) $(CXXFLAGS) $< -o $@

test: $(BINS)
	@for b in $(BINS); do ./$$b || exit 1; done
	@echo "=== SEMUA TEST LULUS ==="

clean:
	rm -rf build

.PHONY: all test clean
