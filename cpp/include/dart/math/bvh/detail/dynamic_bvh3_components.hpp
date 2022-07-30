/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "dart/common/macro.hpp"

namespace dart::math::detail {

//==============================================================================
template <typename Bv_, typename ObjectType = void*>
struct DynamicBvh3Node
{
  using Bv = Bv_;
  using S = typename Bv::S;

  Bv bv;

  union
  {
    DynamicBvh3Node* children[2];
    ObjectType data;
  };

  DynamicBvh3Node* parent;

  DynamicBvh3Node() : parent(nullptr)
  {
    static_assert(sizeof(ObjectType) <= sizeof(DynamicBvh3Node*), "");
    children[0] = nullptr;
    children[1] = nullptr;
  }

  bool is_leaf() const
  {
    return (children[1] == nullptr);
  }

  void set_child(size_t index, DynamicBvh3Node* node)
  {
    children[index] = node;
  }

  void set_left_child(DynamicBvh3Node* node)
  {
    children[0] = node;
    node->parent = this;
  }

  void set_right_child(DynamicBvh3Node* node)
  {
    children[1] = node;
    node->parent = this;
  }

  void set_children(DynamicBvh3Node* left, DynamicBvh3Node* right)
  {
    assert(left);
    assert(right);
    children[0] = left;
    children[1] = right;
    left->parent = this;
    right->parent = this;
  }

  void replace_child(DynamicBvh3Node* old_child, DynamicBvh3Node* new_child)
  {
    if (old_child == children[0]) {
      children[0] = new_child;
    } else {
      assert(old_child == children[1]);
      children[1] = new_child;
    }
  }

  DynamicBvh3Node* get_sibling() const
  {
    assert(parent);
    return (parent->children[1] == this) ? parent->children[0]
                                         : parent->children[1];
  }

  void fit_bv()
  {
    bv = children[0]->bv + children[1]->bv;
  }
};

//==============================================================================
template <typename Node>
struct DefaultSelectPolicy
{
  static Node* run(const Node* target, const Node* new_node)
  {
    using S = typename Node::S;
    using Bv = typename Node::Bv;

    assert(target);
    assert(new_node);
    assert(!target->is_leaf());

    const Bv& bv = new_node->bv;

    const Bv& bv1 = target->children[0]->bv;
    const Bv& bv2 = target->children[1]->bv;

    const Vector3<S> v = bv.getMin() + bv.getMax();
    const Vector3<S> v1 = v - (bv1.getMin() + bv1.getMax());
    const Vector3<S> v2 = v - (bv2.getMin() + bv2.getMax());
    S d1 = std::abs(v1[0]) + std::abs(v1[1]) + std::abs(v1[2]);
    S d2 = std::abs(v2[0]) + std::abs(v2[1]) + std::abs(v2[2]);

    return (d1 < d2) ? target->children[0] : target->children[1];
  }
};

//==============================================================================
template <typename Node>
struct DescendA
{
  static bool descend_a(const Node* node_a, const Node* node_b)
  {
    DART_UNUSED(node_b);
    return not node_a->is_leaf();
  }
};

//==============================================================================
template <typename Node>
struct DescendB
{
  static bool descend_a(const Node* node_a, const Node* node_b)
  {
    DART_UNUSED(node_a);
    return node_b->is_leaf();
  }
};

//==============================================================================
template <typename Node>
struct DescendLarger
{
  static bool descend_a(const Node* node_a, const Node* node_b)
  {
    assert(node_a);
    assert(node_b);
    assert(!node_a->is_leaf() || !node_b->is_leaf());

    if (node_b->is_leaf()) {
      return true;
    }

    if (node_a->is_leaf()) {
      return false;
    }

    return (node_a->bv.size() > node_b->bv.size());
  }
};

} // namespace dart::math::detail
