/*
    Copyright (c) 2013, Taiga Nomi and the respective contributors
    All rights reserved.

    Use of this source code is governed by a BSD-style license that can be found
    in the LICENSE file.
*/
#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "tiny_dnn/layers/layer.h"
#include "tiny_dnn/network.h"
#include "tiny_dnn/node.h"

namespace tiny_dnn {

/**
 * utility for graph visualization
 **/
class graph_visualizer {
 public:
  explicit graph_visualizer(layer *root_node,
                            const std::string &graph_name = "graph")
    : root_(root_node), name_(graph_name) {}

  template <typename N>
  explicit graph_visualizer(network<N> &network,
                            const std::string &graph_name = "graph")
    : root_(network[0]), name_(graph_name) {}

  /**
   * generate graph structure in dot language format
   **/
  void generate(std::ostream &stream) {
    generate_header(stream);
    generate_nodes(stream);
    generate_footer(stream);
  }

 private:
  typedef std::unordered_map<const node *, std::string> node2name_t;

  void generate_header(std::ostream &stream) {
    stream << "digraph \"" << name_ << "\" {" << std::endl;
    stream << "  node [ shape=record ];" << std::endl;
  }

  void generate_nodes(std::ostream &stream) {
    node2name_t node2name;
    get_layer_names(node2name);

    graph_traverse(
      root_, [&](const layer &l) { generate_layer(stream, l, node2name); },
      [&](const edge &e) { generate_edge(stream, e, node2name); });
  }

  void get_layer_names(node2name_t &node2name) {
    std::unordered_map<std::string, int> layer_counts;  // [layer_type -> num]

    auto namer = [&](const layer &l) {
      std::string ltype = l.layer_type();

      // add quote and sequential-id
      node2name[&l] =
        "\"" + ltype + to_string(layer_counts[l.layer_type()]++) + "\"";
    };

    graph_traverse(root_, namer, [&](const edge &) {});
  }

  void generate_edge(std::ostream &stream,
                     const edge &e,
                     node2name_t &node2name) {
    auto next = e.next();
    auto prev = e.prev();

    for (auto n : next) {
      size_t dst_port = n->prev_port(e);
      size_t src_port = prev->next_port(e);
      stream << "  " << node2name[prev] << ":out" << src_port << " -> "
             << node2name[n] << ":in" << dst_port << ";" << std::endl;
    }
  }

  void generate_layer(std::ostream &stream,
                      const layer &layer,
                      node2name_t &node2name) {
    stream << "  " << node2name[&layer] << " [" << std::endl;
    stream << "    label= \"";
    stream << layer.layer_type() << "|{{in";
    generate_layer_channels(stream, layer.in_shape(), layer.in_types(), "in");
    stream << "}|{out";
    generate_layer_channels(stream, layer.out_shape(), layer.out_types(),
                            "out");
    stream << "}}\"" << std::endl;
    stream << "  ];" << std::endl;
  }

  void generate_layer_channels(std::ostream &stream,
                               const std::vector<shape3d> &shapes,
                               const std::vector<vector_type> &vtypes,
                               const std::string &port_prefix) {
    CNN_UNREFERENCED_PARAMETER(vtypes);
    for (size_t i = 0; i < shapes.size(); i++) {
      stream << "|<" << port_prefix << i << ">" << shapes[i] << "(" << vtypes[i]
             << ")";
    }
  }

  void generate_footer(std::ostream &stream) { stream << "}" << std::endl; }

  layer *root_;
  std::string name_;
};

}  // namespace tiny_dnn
