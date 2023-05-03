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

#include <cmath>
#include <mutex>
#include <utility>
#include <vector>

#include <dimod/constrained_quadratic_model.h>

#include "dwave/exceptions.hpp"
#include "dwave/presolve2.hpp"

namespace dwave {
namespace presolve2 {

template <class Bias, class Index, class Assignment>
class PresolverImpl {
 public:
    using model_type = dimod::ConstrainedQuadraticModel<Bias, Index>;

    using bias_type = Bias;
    using index_type = Index;
    using assignment_type = Assignment;
    using size_type = typename model_type::size_type;

    static constexpr double FEASIBILITY_TOLERANCE = 1.0e-6;
    static constexpr double INF = 1.0e30;

    PresolverImpl() = default;
    ~PresolverImpl() = default;

    explicit PresolverImpl(model_type model): flags(), handler_(std::move(model)) {}

    /// Return a const reference to the held constrained quadratic model.
    const model_type& model() const { return handler_.model(); }

    // Normalization Methods --------------------------------------------------

    void normalize() {
        // check for NANs
        normalization_check_nan(handler_.objective());
        for (const auto& constraint : handler_.constraints())
            normalization_check_nan(constraint);
    }

    static void normalization_check_nan(const dimod::Expression<bias_type, index_type>& expression) {
        // We only care about the biases, so let's just cast to the base type for speed
        const dimod::abc::QuadraticModelBase<bias_type, index_type>& base = expression;

        for (auto it = base.cbegin_quadratic(); it != base.cend_quadratic(); ++it) {
            if (std::isnan(it->bias)) throw InvalidModelError("biases cannot be NAN");
        }

        for (size_type v = 0; v < base.num_variables(); ++v) {
            if (std::isnan(base.linear(v))) throw InvalidModelError("biases cannot be NAN");
        }

        if (std::isnan(base.offset())) throw InvalidModelError("biases cannot be NAN");
    }

    static void normalization_flip_constraint(dimod::Constraint<bias_type, index_type>& constraint) {
        if (constraint.sense() == dimod::Sense::GE) constraint.scale(-1);
    }

    // Techniques -------------------------------------------------------------

    static bool technique_remove_small_biases(dimod::Constraint<bias_type, index_type>& expression) {
        // todo : not yet defined for quadratic expressions. A simple approach would be to
        // skip the variables with any interactions. The next level would be to remove the
        // small quadratic biases unconditionally, and then conditionally
        // todo: generic version for expression that doesn't touch the rhs?
        if (!expression.is_linear()) return false;

        static constexpr double CONDITIONAL_REMOVAL_BIAS_LIMIT = 1.0e-3;
        static constexpr double CONDITIONAL_REMOVAL_LIMIT = 1.0e-2;
        static constexpr double UNCONDITIONAL_REMOVAL_BIAS_LIMIT = 1.0e-10;
        static constexpr double SUM_REDUCTION_LIMIT = 1.0e-1;
        std::vector<index_type> small_biases;
        std::vector<index_type> small_biases_temp;
        bias_type reduction = 0;
        bias_type reduction_magnitude = 0;
        for (auto& v : expression.variables()) {
            // ax ◯ c
            bias_type a = expression.linear(v);
            bias_type lb = expression.lower_bound(v);
            bias_type ub = expression.upper_bound(v);
            assert(ub >= lb);
            bias_type v_range = ub - lb;
            if (std::abs(a) < CONDITIONAL_REMOVAL_BIAS_LIMIT &&
                std::abs(a) * v_range * expression.num_variables() <
                        CONDITIONAL_REMOVAL_LIMIT * FEASIBILITY_TOLERANCE) {
                small_biases_temp.emplace_back(v);
                reduction += a * lb;
                reduction_magnitude += std::abs(a) * v_range;
            }
            if (std::abs(a) < UNCONDITIONAL_REMOVAL_BIAS_LIMIT) small_biases.emplace_back(v);
        }
        if (reduction_magnitude < SUM_REDUCTION_LIMIT * FEASIBILITY_TOLERANCE) {
            expression.set_rhs(expression.rhs() - reduction);
            for (auto& u : small_biases_temp) small_biases.emplace_back(u);
        }

        for (auto& v : small_biases) {
            expression.remove_variable(v);
        }
        return small_biases.size();
    }

    static bool technique_remove_zero_biases(dimod::Expression<bias_type, index_type>& expression) {
        // todo: wrap remove_small_biases?
        // quadratic
        std::vector<std::pair<index_type, index_type>> empty_interactions;
        for (auto it = expression.cbegin_quadratic(); it != expression.cend_quadratic(); ++it) {
            if (!(it->bias)) {
                empty_interactions.emplace_back(it->u, it->v);
            }
        }
        for (auto& uv : empty_interactions) {
            expression.remove_interaction(uv.first, uv.second);
        }

        // linear
        std::vector<index_type> empty_variables;
        for (auto& v : expression.variables()) {
            if (expression.linear(v)) continue;
            if (expression.num_interactions(v)) continue;
            empty_variables.emplace_back(v);
        }
        for (auto& v : empty_variables) {
            expression.remove_variable(v);
        }

        return empty_interactions.size() || empty_variables.size();
    }


    TechniqueFlags flags;

 private:
    /// Handles access to the model on behalf of the PresolverImpl. This way
    /// we can maintain a degree of safety.
    /// It also tracks the changes made to Model that will affect the samples
    /// so that we can later retore them.
    class ModelHandler {
     public:
        ModelHandler() = default;
        ~ModelHandler() = default;

        explicit ModelHandler(model_type model): model_(std::move(model)) {}

        /// Return a const reference to the held constrained quadratic model.
        const model_type& model() const { return model_; }

        // We can access the objective and constraints without because
        // modifying them doesn't modify the underlying CQM variables/bounds
        auto& objective() { return model_.objective; }
        const auto& objective() const { return model_.objective; }
        auto constraints() { return model_.constraints(); }
        const auto constraints() const { return model_.constraints(); }

     private:
        model_type model_;

        // todo: mutex to support parallel implementations
    };

    ModelHandler handler_;
};

}  // namespace presolve2
}  // namespace dwave
