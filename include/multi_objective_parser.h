//
// Created by crl on 13/12/2024.
//

#ifndef MULTI_OBJECTIVE_PARSER_H
#define MULTI_OBJECTIVE_PARSER_H
#include <string>

#include "data_structures/adjacency_matrix.h"


class MultiObjectiveParser {
public:
    std::string file_name;

    explicit MultiObjectiveParser(std::string file_name) : file_name(std::move(file_name)) {}

    AdjacencyMatrix parse_graph(const int max_objectives);
};

#endif //MULTI_OBJECTIVE_PARSER_H
