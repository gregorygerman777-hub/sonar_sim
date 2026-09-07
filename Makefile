PYTHON := ./.venv/bin/python

.PHONY: build test demos reproduce clean

build:
	$(PYTHON) setup.py build_ext --inplace

test: build
	$(PYTHON) tests/test_units.py

demos: build
	@for demo in python/demo*.py; do echo "--- $$demo"; $(PYTHON) $$demo; done

# Everything, into a fresh timestamped results directory.
reproduce:
	./run_all.sh

clean:
	rm -rf build build_cmake bindings/sonar.cpp *.so python/__pycache__
