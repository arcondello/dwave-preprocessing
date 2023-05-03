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

#include <limits>

#include <catch2/catch.hpp>

// #include "dimod/constrained_quadratic_model.h"
// #include "dwave/presolve.h"

// #include "presolve2.hpp"
#include "presolveimpl.hpp"

namespace dwave {

using PresolverImpl = presolve2::PresolverImpl<double, int, double>;
using ConstrainedQuadraticModel = dimod::ConstrainedQuadraticModel<double, int>;

TEST_CASE("Test construction", "[presolve][impl]") {
    SECTION("Default constructor") {
        auto pre = PresolverImpl();
        CHECK_FALSE(pre.flags);
        CHECK_FALSE(pre.model().num_variables());
        CHECK_FALSE(pre.model().num_constraints());
    }

    SECTION("Empty model") {
        auto pre = PresolverImpl(ConstrainedQuadraticModel());
        CHECK_FALSE(pre.flags);
        CHECK_FALSE(pre.model().num_variables());
        CHECK_FALSE(pre.model().num_constraints());
    }
}

TEST_CASE("Test normalization_check_nan", "[presolve][impl]") {
    SECTION("Linear objective with NaNs") {
        auto cqm = ConstrainedQuadraticModel();
        cqm.add_variable(dimod::BINARY);
        cqm.objective.set_linear(0, std::numeric_limits<double>::quiet_NaN());
        CHECK_THROWS_AS(PresolverImpl::normalization_check_nan(cqm.objective),
                        presolve2::InvalidModelError);
    }
}

TEST_CASE("Test normalization_flip_constraints", "[presolve][impl]") {
    GIVEN("A cqm with three constraints with different senses") {
        auto cqm = ConstrainedQuadraticModel();
        cqm.add_variables(dimod::Vartype::BINARY, 3);
        auto c0 = cqm.add_linear_constraint({0, 1}, {1, 1}, dimod::Sense::EQ, 1);
        auto c1 = cqm.add_linear_constraint({0, 1}, {1, 1}, dimod::Sense::LE, 1);
        auto c2 = cqm.add_linear_constraint({0, 1}, {1, 1}, dimod::Sense::GE, 1);

        WHEN("We apply normalization_flip_constraints to the EQ constraint") {
            auto& constraint = cqm.constraint_ref(c0);
            PresolverImpl::normalization_flip_constraint(constraint);

            THEN("The EQ constraint is not flipped") {
                REQUIRE(constraint.num_variables() == 2);
                CHECK(constraint.is_linear());

                CHECK(constraint.linear(0) == 1);
                CHECK(constraint.linear(1) == 1);
                CHECK(constraint.sense() == dimod::Sense::EQ);
                CHECK(constraint.rhs() == 1);
            }
        }

        WHEN("We apply normalization_flip_constraints to the LE constraint") {
            auto& constraint = cqm.constraint_ref(c1);
            PresolverImpl::normalization_flip_constraint(constraint);

            THEN("The LE constraint is not flipped") {
                REQUIRE(constraint.num_variables() == 2);
                CHECK(constraint.is_linear());

                CHECK(constraint.linear(0) == 1);
                CHECK(constraint.linear(1) == 1);
                CHECK(constraint.sense() == dimod::Sense::LE);
                CHECK(constraint.rhs() == 1);
            }
        }

        WHEN("We apply normalization_flip_constraints to the GE constraint") {
            auto& constraint = cqm.constraint_ref(c2);
            PresolverImpl::normalization_flip_constraint(constraint);

            THEN("The GE constraint is flipped") {
                REQUIRE(constraint.num_variables() == 2);
                CHECK(constraint.is_linear());

                CHECK(constraint.linear(0) == -1);
                CHECK(constraint.linear(1) == -1);
                CHECK(constraint.sense() == dimod::Sense::LE);
                CHECK(constraint.rhs() == -1);
            }
        }
    }
}

TEST_CASE("Test technique_remove_small_biases", "[presolve][impl]") {
    GIVEN("An empty constraint") {
        auto cqm = ConstrainedQuadraticModel();
        auto constraint = cqm.new_constraint();

        WHEN("We apply remove_small_biases to it") {
            PresolverImpl::technique_remove_small_biases(constraint);

            THEN("Nothing changes") {
                CHECK(constraint.num_variables() == 0);
            }
        }
    }

    // todo: more tests, quadratic, etc
}

TEST_CASE("Test technique_remove_zero_biases", "[presolve][impl]") {
    GIVEN("A quadratic constraint with all zero biases") {
        auto cqm = ConstrainedQuadraticModel();
        cqm.add_variables(dimod::BINARY, 2);

        auto constraint = cqm.new_constraint();
        constraint.set_quadratic(0, 1, 0);
        constraint.set_sense(dimod::Sense::GE);

        WHEN("We apply remove_small_biases to it") {
            PresolverImpl::technique_remove_zero_biases(constraint);

            THEN("Everything is removed") {
                CHECK(constraint.num_variables() == 0);
            }
        }
    }

    GIVEN("A quadratic constraint with quadratic 0 biases, but non-zero linear biases") {
        auto cqm = ConstrainedQuadraticModel();
        cqm.add_variables(dimod::BINARY, 2);

        auto constraint = cqm.new_constraint();
        constraint.set_quadratic(0, 1, 0);
        constraint.set_sense(dimod::Sense::GE);
        constraint.set_linear(1, 1.5);

        WHEN("We apply remove_small_biases to it") {
            PresolverImpl::technique_remove_zero_biases(constraint);

            THEN("One variable is removed") {
                REQUIRE(constraint.num_variables() == 1);
                CHECK(constraint.linear(1) == 1.5);
                CHECK(constraint.is_linear());
            }
        }
    }
}

}  // namespace dwave
