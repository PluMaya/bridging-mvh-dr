//
// Created by crl on 15/02/2026.
//

#ifndef MULTI_VALUED_HEURISTIC_PARSER_H
#define MULTI_VALUED_HEURISTIC_PARSER_H

#include <definitions.h>

class MVHParser {
public:

    explicit MVHParser() = default;

    static MultiValuedHeuristic parse_heuristic(const std::string& heuristic_path);
};

#endif //MULTI_VALUED_HEURISTIC_PARSER_H
