//
// Created by crl on 15/02/2026.
//

#ifndef SINGLE_VALUED_HEURISTIC_PARSER_H
#define SINGLE_VALUED_HEURISTIC_PARSER_H
#include <string>
#include <utility>

#include "definitions.h"


class SVHParser {
public:

    explicit SVHParser() = default;

    static Heuristic parse_heuristic(const std::string& heuristic_path);
};

#endif //SINGLE_VALUED_HEURISTIC_PARSER_H
