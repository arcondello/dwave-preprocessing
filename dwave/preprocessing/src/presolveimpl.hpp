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

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dimod/constrained_quadratic_model.h"
#include "dwave/exceptions.hpp"
#include "dwave/flags.hpp"

namespace dwave {
namespace presolve {

template <class Bias, class Index = int, class Assignment = double>
class PresolverImpl {
 public:
    using model_type = dimod::ConstrainedQuadraticModel<Bias, Index>;
    using constraint_type = dimod::Constraint<Bias, Index>;
    using expression_type = dimod::Expression<Bias, Index>;

    using bias_type = Bias;
    using index_type = Index;
    using size_type = typename model_type::size_type;

    using assignment_type = Assignment;

    static constexpr double FEASIBILITY_TOLERANCE = 1.0e-6;
    static constexpr double INF = 1.0e30;

    /// Default constructor.
    PresolverImpl() = default;

    /// Default destructor.
    ~PresolverImpl() = default;

    /// Construct a presolver from a constrained quadratic model.
    explicit PresolverImpl(model_type model) : model_(std::move(model)) {}

    /// Apply any loaded presolve techniques. Acts on the model() in-place.
    bool apply() {
        return normalize() | presolve();
    }

    /// Detach the constrained quadratic model and return it.
    /// This clears the model from the presolver.
    model_type detach_model() {
        detached_ = true;
        return model_.detach_model();
    }

    const Feasibility& feasibility() const { return feasibility_; }

    /// Return a const reference to the held constrained quadratic model.
    const model_type& model() const { return model_.model(); }

    bool normalize() {
        if (detached_) {
            throw std::logic_error(
                    "model has been detached, so there is no model to apply presolve() to");
        }

        bool changes = false;

        changes |= normalization_check_nan();
        changes |= normalization_spin_to_binary();
        changes |= normalization_remove_offsets();
        changes |= normalization_remove_self_loops();
        changes |= normalization_flip_constraints();
        changes |= normalization_remove_invalid_markers();

        normalized_ = true;

        return changes;
    }

    bool normalization_check_nan() const {
        bool changes = normalization_check_nan(model_.objective());
        for (auto& constraint : model_.constraints()) {
            changes |= normalization_check_nan(constraint);
        }
        return changes;
    }

    static bool normalization_check_nan(const dimod::Expression<bias_type, index_type>& expression) {
        // We only care about the biases, so let's just cast to the base type for speed
        const dimod::abc::QuadraticModelBase<bias_type, index_type>& base = expression;

        for (auto it = base.cbegin_quadratic(); it != base.cend_quadratic(); ++it) {
            if (std::isnan(it->bias)) throw InvalidModelError("biases cannot be NAN");
        }

        for (size_type v = 0; v < base.num_variables(); ++v) {
            if (std::isnan(base.linear(v))) throw InvalidModelError("biases cannot be NAN");
        }

        if (std::isnan(base.offset())) {
            throw InvalidModelError("biases cannot be NAN");
        }

        return false;
    }

    /// Convert any >= constraints into <=.
    bool normalization_flip_constraints() {
        bool changes = false;
        for (auto& constraint : model_.constraints()) {
            changes |= normalization_flip_constraint(constraint);
        }
        return changes;
    }

    /// Convert a >= constraint into <=.
    static bool normalization_flip_constraint(constraint_type& constraint) {
        if (constraint.sense() == dimod::Sense::GE) {
            constraint.scale(-1);
            return true;   // we made changes
        } else {
            return false;  // no changes made
        }
    }

    bool normalization_remove_invalid_markers() {
        bool changes = false;

        std::vector<index_type> discrete;
        for (size_type c = 0; c < model_.num_constraints(); ++c) {
            auto& constraint = model_.constraint_ref(c);

            if (!constraint.marked_discrete()) {
                continue;
            }

            // we can check if it's well formed
            if (constraint.is_onehot()) {
                discrete.push_back(c);
            } else {
                constraint.mark_discrete(false);  // if it's not one-hot, it's not discrete
                changes = true;
            }
        }
        // check if they overlap
        size_type i = 0;
        while (i < discrete.size()) {
            // check if ci overlaps with any other constraints
            auto& constraint = model_.constraint_ref(discrete[i]);

            bool overlap = false;
            for (size_type j = i + 1; j < discrete.size(); ++j) {
                if (model_.constraint_ref(discrete[j]).shares_variables(constraint)) {
                    // we have overlap!
                    overlap = true;
                    constraint.mark_discrete(false);
                    changes = true;
                    break;
                }
            }

            if (overlap) {
                discrete.erase(discrete.begin() + i);
                continue;
            }

            ++i;
        }

        return changes;
    }

    /// Remove the offsets from all constraints in the model.
    bool normalization_remove_offsets() {
        bool changes = false;
        for (auto& constraint : model_.constraints()) {
            changes |= normalization_remove_offset(constraint);
        }
        return changes;
    }

    /// Remove the offset from a cosntraint. E.g. `x + 1 <= 2` becomes `x <= 1`.
    static bool normalization_remove_offset(constraint_type& constraint) {
        if (constraint.offset()) {
            constraint.set_rhs(constraint.rhs() - constraint.offset());
            constraint.set_offset(0);
            return true;   // we made changes
        } else {
            return false;  // no changes made
        }
    }

    /// Remove any self-loops (e.g. x^2) by adding a new variable and an equality constraint.
    bool normalization_remove_self_loops() {
        std::unordered_map<index_type, index_type> mapping;

        // Function to go through the objective/constraints and for each variable in a self-loop,
        // we add a new variable it can interact with.
        // We could break this out into a standalone method, but because all of  those methods
        // would need to share a common mapping, there isn't really an opportunity for
        // parallization. So let's keep it contained for now.
        auto substitute = [&](expression_type& expression) {
            std::size_t current_num_variables = expression.num_variables();
            for (std::size_t i = 0; i < current_num_variables; ++i) {
                index_type v = expression.variables()[i];

                if (!expression.has_interaction(v, v)) {
                    // no self-loop
                    continue;
                }

                auto it = mapping.find(v);
                if (it == mapping.end()) {
                    // we haven't seen this variable before
                    it = mapping.emplace_hint(
                            it, v,
                            model_.add_variable(model_.vartype(v), model_.lower_bound(v),
                                                model_.upper_bound(v)));
                }

                assert(it != mapping.end());

                // Set the bias between v and the new variable
                // We could spread the linear bias out between the two variables, but in that case
                // we would probably want to do that in every expression for consistency and this
                // is a lot easier. But it's something we could test in the future
                expression.add_quadratic(v, it->second, expression.quadratic(v, v));
                expression.remove_interaction(v, v);
            }
        };

        // Actually apply the method
        substitute(model_.objective());
        for (auto& constraint : model_.constraints()) {
            substitute(constraint);
        }

        // We add the new constraints last, because otherwise we would cause reallocation
        for (auto& uv : mapping) {
            model_.add_linear_constraint({uv.first, uv.second}, {+1, -1}, dimod::Sense::EQ, 0);
        }

        return mapping.size();
    }

    /// Convert any SPIN variables to BINARY variables
    bool normalization_spin_to_binary() {
        bool changes = false;
        for (size_type v = 0; v < model_.num_variables(); ++v) {
            if (model_.vartype(v) == dimod::Vartype::SPIN) {
                model_.change_vartype(dimod::Vartype::BINARY, v);
                changes = true;
            }
        }
        return changes;
    }

    bool presolve() {
        if (detached_) {
            throw std::logic_error(
                    "model has been detached, so there is no model to apply presolve() to");
        }
        if (!normalized_) {
            throw std::logic_error("model must be normalized before presolve() is applied");
        }

        // If no techniques have been loaded, return early.
        if (!techniques) {
            return false;
        }

        bool changes = true;
        const index_type max_num_rounds = 100;  // todo: make configurable
        for (index_type num_rounds = 0; num_rounds < max_num_rounds; ++num_rounds) {
            if (!changes) {
                break;
            }
            changes = false;

            // *-- clear out 0 variables/interactions in the constraints and objective
            changes |= technique_remove_zero_biases();
            // *-- clear out small linear biases in the constraints
            changes |= technique_remove_small_biases();
            // *-- remove single variable constraints
            changes |= technique_remove_single_variable_constraints();
            // *-- tighten bounds based on vartype
            changes |= technique_tighten_bounds();
            // *-- domain propagation
            changes |= technique_domain_propagation();
            // *-- remove variables that are fixed by bounds
            changes |= technique_remove_fixed_variables();
        }

        // Cleanup
        // There are a few normalization steps we want to re-run to clean up the model
        // e.g. discrete variable markers may not be valid anymore.
        // Todo: we could replace this with a step where we can also add discrete markers
        changes |= normalization_remove_invalid_markers();

        // Sanity check that we didn't break normalization
        assert(!normalize());

        return changes;
    }

    /// Return a sample of the original CQM from a sample of the reduced CQM.
    template<class T>
    std::vector<T> restore(std::vector<T> reduced) const {
        return model_.restore(std::move(reduced));
    }

    TechniqueFlags techniques = TechniqueFlags::None;

 private:

    // We want to control access to the model in order to track changes,
    // so we create a rump version of the model.
    class ModelView : private model_type {
     public:
        ModelView() = default;
        ~ModelView() = default;

        explicit ModelView(model_type&& model) : model_type(model) {}

        // Const methods are all safe to expose
        using model_type::num_constraints;
        using model_type::num_variables;
        using model_type::lower_bound;
        using model_type::upper_bound;
        using model_type::vartype;

        // We can expose the expression access methods because changes to
        // the individual expressions don't get tracked
        using model_type::constraint_ref;
        using model_type::constraints;

        // Adding/removing constraints isn't tracked
        using model_type::add_linear_constraint;
        using model_type::remove_constraint;

        // The bound changes don't get tracked
        using model_type::set_lower_bound;
        using model_type::set_upper_bound;

        // Expose the objective. Changes don't get tracked
        auto& objective() { return static_cast<model_type*>(this)->objective; }
        const auto& objective() const { return static_cast<const model_type*>(this)->objective; }

        // Expose the model as a const reference
        const model_type& model() const { return *this; }

        // Clear the model, but not the transforms queue.
        model_type detach_model() {
            using std::swap;  // ADL, though doubt it makes a difference
            auto cqm = dimod::ConstrainedQuadraticModel<bias_type, index_type>();
            swap(*this, cqm);
            return cqm;
        }

        // Track when we add a variable to the model.
        index_type add_variable(dimod::Vartype vartype, bias_type lb, bias_type ub) {
            index_type v = model_type::add_variable(vartype, lb, ub);
            transforms_.emplace_back(TransformKind::ADD);
            transforms_.back().v = v;
            return v;
        }

        // Track when we change the vartype of a variable
        void change_vartype(dimod::Vartype vartype, index_type v) {
            if ((model_type::vartype(v) == dimod::Vartype::SPIN) &&
                (vartype == dimod::Vartype::BINARY)) {
                // SPIN->BINARY
                transforms_.emplace_back(TransformKind::SUBSTITUTE);
                transforms_.back().v = v;
                transforms_.back().multiplier = 2;
                transforms_.back().offset = -1;
            } else {
                // We currently only need SPIN->BINARY, but can add more as needed
                throw std::logic_error("unsupported vartype change");
            }
            model_type::change_vartype(vartype, v);
        }

        // Track when we remove a variable from the model by fixing it
        void fix_variable(index_type v, assignment_type assignment) {
            model_type::fix_variable(v, assignment);

            transforms_.emplace_back(TransformKind::FIX);
            transforms_.back().v = v;
            transforms_.back().value = assignment;
        }

        // Restore a sample by undoing all of the transforms
        template <class T>
        std::vector<T> restore(std::vector<T> sample) const {
            // all that we have to do is undo the transforms back to front.
            for (auto it = transforms_.crbegin(); it != transforms_.crend(); ++it) {
                switch (it->kind) {
                    case TransformKind::FIX:
                        sample.insert(sample.begin() + it->v, it->value);
                        break;
                    case TransformKind::SUBSTITUTE:
                        sample[it->v] *= it->multiplier;
                        sample[it->v] += it->offset;
                        break;
                    case TransformKind::ADD:
                        sample.erase(sample.begin() + it->v);
                        break;
                }
            }
            return sample;
        }

     private:
        // we want to track what changes were made
        enum TransformKind { FIX, SUBSTITUTE, ADD };

        // We could get fancy with pointers and templates to save a bit of space here
        struct Transform {
            TransformKind kind;
            index_type v;           // what variable it was applied to
            assignment_type value;  // if it was fixed what it was fixed to
            bias_type multiplier;
            bias_type offset;

            explicit Transform(TransformKind kind)
                    : kind(kind), v(-1), value(NAN), multiplier(NAN), offset(NAN) {}
        };

        std::vector<Transform> transforms_;
    };

    ModelView model_;

    bool detached_ = false;
    bool normalized_ = false;

    Feasibility feasibility_ = Feasibility::Unknown;

    //----- Trivial Techniques -----//

    bool technique_remove_single_variable_constraints() {
        bool ret = false;
        size_type c = 0;
        while (c < model_.num_constraints()) {
            auto& constraint = model_.constraint_ref(c);

            if (constraint.num_variables() == 0) {
                if (!constraint.is_soft()) {
                    // check feasibity
                    switch (constraint.sense()) {
                        case dimod::Sense::EQ:
                            if (constraint.offset() != constraint.rhs()) {
                                // need this exact message for Python
                                throw std::logic_error("infeasible");
                            }
                            break;
                        case dimod::Sense::LE:
                            if (constraint.offset() > constraint.rhs()) {
                                // need this exact message for Python
                                throw std::logic_error("infeasible");
                            }
                            break;
                        case dimod::Sense::GE:
                            if (constraint.offset() < constraint.rhs()) {
                                // need this exact message for Python
                                throw std::logic_error("infeasible");
                            }
                            break;
                    }
                }

                // we remove the constraint regardless of whether it's soft
                // or not. We could use the opportunity to update the objective
                // offset with the violation of the soft constraint, but
                // presolve does not preserve the energy in general, so it's
                // better to avoid side effects and just remove.
                model_.remove_constraint(c);
                ret = true;
                continue;
            } else if (constraint.num_variables() == 1 && !constraint.is_soft()) {
                index_type v = constraint.variables()[0];

                // ax ◯ c
                bias_type a = constraint.linear(v);
                assert(a);  // should have already been removed if 0

                // offset should have already been removed but may as well be safe
                bias_type rhs = (constraint.rhs() - constraint.offset()) / a;

                // todo: test if negative

                if (constraint.sense() == dimod::Sense::EQ) {
                    model_.set_lower_bound(v, std::max(rhs, model_.lower_bound(v)));
                    model_.set_upper_bound(v, std::min(rhs, model_.upper_bound(v)));
                } else if ((constraint.sense() == dimod::Sense::LE) != (a < 0)) {
                    model_.set_upper_bound(v, std::min(rhs, model_.upper_bound(v)));
                } else {
                    assert((constraint.sense() == dimod::Sense::GE) == (a >= 0));
                    model_.set_lower_bound(v, std::max(rhs, model_.lower_bound(v)));
                }

                model_.remove_constraint(c);
                ret = true;
                continue;
            }

            ++c;
        }
        return ret;
    }
    bool technique_remove_zero_biases() {
        bool ret = false;

        ret |= remove_zero_biases(model_.objective());
        for (size_type c = 0; c < model_.num_constraints(); ++c) {
            ret |= remove_zero_biases(model_.constraint_ref(c));
        }

        return ret;
    }
    bool technique_remove_small_biases() {
        bool ret = false;

        for (size_type c = 0; c < model_.num_constraints(); ++c) {
            ret |= remove_small_biases(model_.constraint_ref(c));
        }

        return ret;
    }
    bool technique_tighten_bounds() {
        bool ret = false;
        bias_type lb;
        bias_type ub;
        for (size_type v = 0; v < model_.num_variables(); ++v) {
            switch (model_.vartype(v)) {
                case dimod::Vartype::SPIN:
                case dimod::Vartype::BINARY:
                case dimod::Vartype::INTEGER:
                    ub = model_.upper_bound(v);
                    if (ub != std::floor(ub)) {
                        model_.set_upper_bound(v, std::floor(ub));
                        ret = true;
                    }
                    lb = model_.lower_bound(v);
                    if (lb != std::ceil(lb)) {
                        model_.set_lower_bound(v, std::ceil(lb));
                        ret = true;
                    }
                    break;
                case dimod::Vartype::REAL:
                    break;
            }
        }
        return ret;
    }
    bool technique_domain_propagation() {
        bool ret = false;

        for (size_type c = 0; c < model_.num_constraints(); ++c) {
            ret |= domain_propagation(model_.constraint_ref(c));
        }

        return ret;
    }
    bool technique_remove_fixed_variables() {
        bool ret = false;
        size_type v = 0;
        while (v < model_.num_variables()) {
            if (model_.lower_bound(v) == model_.upper_bound(v)) {
                model_.fix_variable(v, model_.lower_bound(v));
                ret = true;
            }
            ++v;
        }
        return ret;
    }
    static bool remove_zero_biases(dimod::Expression<bias_type, index_type>& expression) {
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
    static bool remove_small_biases(dimod::Constraint<bias_type, index_type>& expression) {
        // todo : not yet defined for quadratic expressions
        if (!expression.is_linear()) {
            return false;
        }

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
            if (std::abs(a) < UNCONDITIONAL_REMOVAL_BIAS_LIMIT)
                small_biases.emplace_back(v);
        }
        if (reduction_magnitude < SUM_REDUCTION_LIMIT * FEASIBILITY_TOLERANCE) {
            expression.set_rhs(expression.rhs() - reduction);
            for (auto& u : small_biases_temp) {
                small_biases.emplace_back(u);
            }
        }

        for (auto& v : small_biases) {
            expression.remove_variable(v);
        }
        return small_biases.size();
    }
    static std::pair<bias_type, bias_type> get_min_max_activities(
    dimod::Constraint<bias_type, index_type>& expression, int exclude_v) {
        std::pair<bias_type, bias_type> result;
        bias_type minac = 0;
        bias_type maxac = 0;
        for (auto& v : expression.variables()) {
            if (v == exclude_v) continue;

            bias_type a = expression.linear(v);
            bias_type lb = expression.lower_bound(v);
            bias_type ub = expression.upper_bound(v);
            assert(ub >= lb);

            if (a > 0) {
                if (lb > - INF) minac += a * lb;
                else minac = -INF;
                if (ub < INF) maxac += a * ub;
                else maxac = INF;
            }
            else {
                if (ub < INF) minac += a * ub;
                else minac = -INF;
                if (lb > -INF) maxac += a * lb;
                else maxac = INF;
            }
        }
        result.first = minac;
        result.second = maxac;
        return result;
    }
    bool domain_propagation(dimod::Constraint<bias_type, index_type>& expression) {
        // only defined for linear constraints
        if (!expression.is_linear() || expression.is_soft()) {
            return false;
        }

        static constexpr double NEW_BOUND_MAX = 1.0e8;
        static constexpr double MIN_CHANGE_FOR_BOUND_UPDATE = 1.0e-3;
        bool continue_domain_propagation = false;
        // equality_constraints should be broken to two inequalities
        bool equality_constraint = false;
        if (expression.sense() == dimod::Sense::EQ) {
            equality_constraint = true;
        }

        for (auto& v : expression.variables()) {
            // domain propagation does not apply to binary variable
            if (model_.vartype(v) == dimod::Vartype::BINARY) {
                continue;
            }

            // todo: reduce the calls to 'get_min_max_activities' by calling
            // todo: it once for each constraint and not each variable
            std::pair<bias_type, bias_type> result = get_min_max_activities(expression, v);
            auto& minac = result.first;
            auto& maxac = result.second;

            bias_type a = expression.linear(v);
            bias_type lb = expression.lower_bound(v);
            bias_type ub = expression.upper_bound(v);
            assert(ub >= lb);

            // calculate the potential new bounds
            bias_type pnb1 = (expression.rhs() - minac)/a; // for LE and EQ constraints
            bias_type pnb2 = (expression.rhs() - maxac)/a; // only for EQ constraints
            if (std::abs(pnb1) > NEW_BOUND_MAX) {
                continue;
            }
            if (equality_constraint && std::abs(pnb2) > NEW_BOUND_MAX) {
                continue;
            }

            if (a > 0) {
                if (minac > -INF && expression.rhs() < INF &&
                    ub - pnb1 > MIN_CHANGE_FOR_BOUND_UPDATE * FEASIBILITY_TOLERANCE) {
                    if (pnb1 > lb && pnb1 < ub) {
                        model_.set_upper_bound(v, pnb1);
                        continue_domain_propagation = true;
                    } else if (pnb1 < lb)
                        throw std::logic_error("infeasible");
                }
                if (equality_constraint && maxac < INF && expression.rhs() > -INF &&
                    pnb2 - lb > MIN_CHANGE_FOR_BOUND_UPDATE * FEASIBILITY_TOLERANCE) {
                    if (pnb2 > lb && pnb2 < ub) {
                        model_.set_lower_bound(v, pnb2);
                        continue_domain_propagation = true;
                    } else if (pnb2 > ub) {
                        throw std::logic_error("infeasible");
                    }
                }
            }
            if (a < 0) {
                if (minac > -INF && expression.rhs() < INF &&
                    pnb1 - lb > MIN_CHANGE_FOR_BOUND_UPDATE * FEASIBILITY_TOLERANCE) {
                    if (pnb1 > lb && pnb1 < ub) {
                        model_.set_lower_bound(v, pnb1);
                        continue_domain_propagation = true;
                    } else if (pnb1 > ub) {
                        throw std::logic_error("infeasible");
                    }
                }
                if (equality_constraint && maxac < INF && expression.rhs() > -INF &&
                    ub - pnb2 > MIN_CHANGE_FOR_BOUND_UPDATE * FEASIBILITY_TOLERANCE) {
                    if (pnb2 > lb && pnb2 < ub) {
                        model_.set_upper_bound(v, pnb2);
                        continue_domain_propagation = true;
                    } else if (pnb2 < lb) {
                        throw std::logic_error("infeasible");
                    }
                }
            }
        }
        return continue_domain_propagation;
    }
};
}  // namespace presolve
}  // namespace dwave
