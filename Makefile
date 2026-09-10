PYTHON := ./.venv/bin/python

.PHONY: build test demos reproduce clean

build:
	$(PYTHON) setup.py build_ext --inplace

test: build
	$(PYTHON) tests/test_units.py
	$(PYTHON) tests/test_research.py

demos: build
	@for demo in python/demo*.py; do $(PYTHON) $$demo || exit $$?; done

# Everything, into a fresh timestamped results directory.
reproduce:
	./run_all.sh

clean:
	rm -rf build build_cmake bindings/sonar.cpp *.so python/__pycache__

console: build
	$(PYTHON) python/research_console.py
