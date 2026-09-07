import os

from setuptools import setup, Extension
from Cython.Build import cythonize

eigen = next((p for p in ("/opt/homebrew/include/eigen3", "/usr/include/eigen3",
                          "/usr/local/include/eigen3") if os.path.isdir(p)), None)

extension = Extension(
    "sonar",
    sources=["bindings/sonar.pyx", "core/geometry.cpp", "core/physics.cpp",
             "core/simulator.cpp", "core/camera.cpp", "core/waveform.cpp"],
    include_dirs=["core"] + ([eigen] if eigen else []),
    language="c++",
    extra_compile_args=["-std=c++17", "-O3", "-pthread"],
    extra_link_args=["-pthread"],
)

setup(name="sonar", ext_modules=cythonize([extension], language_level=3))
