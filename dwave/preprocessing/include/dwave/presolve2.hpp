// Copyright 2022 D-Wave Systems Inc.
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#pragma once

#include <experimental/propagate_const>
#include <memory>
#include <cstdint>

#include "dimod/constrained_quadratic_model.h"

namespace dwave {
namespace presolve2 {

enum TechniqueFlags: std::uint64_t {
    Technique1 = 1 << 0,
    Technique2 = 1 << 1,

    Normalization = Technique1 | Technique2,

    All = 0xffffffffffffffffu
};


template <class Bias, class Index, class Assignment>
class Presolver {
 public:
    Presolver();
    ~Presolver();

    explicit Presolver(dimod::ConstrainedQuadraticModel<Bias, Index> model);

    void load_default_presolvers();

    void normalize();

 protected:
    class PresolverImpl_;
    std::experimental::propagate_const<std::unique_ptr<PresolverImpl_>> impl_;
};

}  // namespace presolve2
}  // namespace dwave
