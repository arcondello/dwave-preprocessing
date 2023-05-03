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
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or PresolverImplied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#include "dwave/presolve2.hpp"

#include "presolveimpl.hpp"

namespace dwave {
namespace presolve2 {

template <class Bias, class Index, class Assignment>
class Presolver<Bias, Index, Assignment>::PresolverImpl_
        : public PresolverImpl<Bias, Index, Assignment> {
    using PresolverImpl<Bias, Index, Assignment>::PresolverImpl;  // inherit constructors as well
};

template <class Bias, class Index, class Assignment>
Presolver<Bias, Index, Assignment>::Presolver() : impl_(new PresolverImpl_) {}

template <class Bias, class Index, class Assignment>
Presolver<Bias, Index, Assignment>::~Presolver() = default;

template <class Bias, class Index, class Assignment>
Presolver<Bias, Index, Assignment>::Presolver(dimod::ConstrainedQuadraticModel<Bias, Index> model)
        : impl_(std::make_unique<PresolverImpl_>(std::move(model))) {}

template <class Bias, class Index, class Assignment>
void Presolver<Bias, Index, Assignment>::load_default_presolvers() {
    impl_->flags = TechniqueFlags::All;
}

template <class Bias, class Index, class Assignment>
void Presolver<Bias, Index, Assignment>::normalize() {
    impl_->normalize();
}

// If we start seeing pickup on this library, we can build other numeric
// combinations. But for now, this is the only CQM combo that dimod distributes.
template class Presolver<double, int, double>;

}  // namespace presolve2
}  // namespace dwave
