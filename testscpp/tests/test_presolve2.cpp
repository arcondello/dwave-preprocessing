// Copyright 2023 D-Wave Systems Inc.
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

#include <cmath>

#include <catch2/catch.hpp>

#include "dimod/constrained_quadratic_model.h"
#include "dwave/exceptions.hpp"
#include "dwave/presolve2.hpp"

namespace dwave {

using Presolver = presolve2::Presolver<double, int, double>;
using ConstrainedQuadraticModel = dimod::ConstrainedQuadraticModel<double, int>;

TEST_CASE("Presolver can be constructed", "[presolve]") {
    auto pre = presolve2::Presolver<double, int, double>();
}

TEST_CASE("Models with NANs raise an exception", "[presolve]") {
    GIVEN("A CQM with NANs in the objective's linear biases") {
        auto cqm = ConstrainedQuadraticModel();
        auto v = cqm.add_variable(dimod::Vartype::BINARY);
        cqm.objective.set_linear(v, NAN);

        THEN("Normalizing raises an error") {
            auto pre = Presolver(cqm);
            CHECK_THROWS_AS(pre.normalize(), presolve2::InvalidModelError);
        }
    }
}

}  // namespace dwave
