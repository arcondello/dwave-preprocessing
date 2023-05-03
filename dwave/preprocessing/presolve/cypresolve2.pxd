# distutils: language = c++
# distutils: include_dirs = dwave/preprocessing/include/
# cython: language_level=3

# Copyright 2023 D-Wave Systems Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cdef extern from "dwave/presolve2.hpp" namespace "dwave::presolve2" nogil:

    cdef cppclass cppPresolver "dwave::presolve2::Presolver" [Bias, Index, Assignment]:
        Presolver2()
        void load_default_presolvers()


cdef class Presolver:
    cdef cppPresolver[double, int, double] cpppresolver
