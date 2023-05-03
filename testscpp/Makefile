ROOT := ..
INCLUDE := $(ROOT)/dwave/preprocessing/include
SRC := $(ROOT)/dwave/preprocessing/src
DIMOD := $(shell python -c 'import dimod; print(dimod.get_include())')
CATCH2 := $(ROOT)/testscpp/Catch2/single_include/
FLAGS := -std=c++17 -Wall -Wno-unknown-pragmas -fcompare-debug-second -O3

test_main.o:
	$(CXX) test_main.cpp -c

# Developer note: we could make this more generic, but because the package structure
# is not at all consistent and because we have so few test files I think it's clearer to
# just enumerate them for now.

presolve2.o: $(SRC)/presolve2.cpp $(SRC)/presolveimpl.hpp $(INCLUDE)/dwave/presolve2.hpp
	$(CXX) $(FLAGS) $(SRC)/presolve2.cpp -c -I$(INCLUDE) -I$(DIMOD)

test_presolve2.o: tests/test_presolve2.cpp
	$(CXX) $(FLAGS) tests/test_presolve2.cpp -c -I$(CATCH2) -I$(INCLUDE) -I$(DIMOD)

test_presolveimpl.o: tests/test_presolveimpl.cpp $(SRC)/presolveimpl.hpp
	$(CXX) $(FLAGS) tests/test_presolveimpl.cpp -c -I$(CATCH2) -I$(INCLUDE) -I$(SRC)  -I$(DIMOD)

test_roof_duality.o: tests/test_roof_duality.cpp $(wildcard $(INCLUDE)/dwave-preprocessing/*hpp)
	$(CXX) $(FLAGS) tests/test_roof_duality.cpp -c -I$(DIMOD) -I$(CATCH2) -I$(INCLUDE)

tests.out: test_main.o test_presolve2.o presolve2.o test_presolveimpl.o test_roof_duality.o
	$(CXX) $(FLAGS) test_main.o test_presolve2.o presolve2.o test_roof_duality.o test_presolveimpl.o -o tests.out

tests: tests.out
	./tests.out

clean:
	rm *.o *.out
