//
// Created by crl on 13/07/2024.
//

#include "parser.h"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>

namespace bfs = boost::filesystem;

// ── Helpers ───────────────────────────────────────────────────────────────────

// Parses a DIMACS .gr line of the form "a <src> <tgt> <cost>".
// Returns type='a' for edge lines; type='c'/'p'/'\0' for everything else.
struct GrLine {
    char   type = '\0';
    size_t src  = 0;
    size_t tgt  = 0;
    unsigned long cost = 0;
};

static GrLine parse_gr_line(const std::string &line) {
    GrLine result;
    const char *p = line.c_str();
    while (*p == ' ' || *p == '\t') ++p;
    result.type = *p;
    if (*p != 'a') return result;
    char *end;
    result.src  = std::strtoul(p + 1, &end, 10);
    result.tgt  = std::strtoul(end,   &end, 10);
    result.cost = std::strtoul(end,   nullptr, 10);
    return result;
}

// ── Multi-file parser (one *.gr file per objective) ───────────────────────────

static AdjacencyMatrix parse_multi_file(const std::string &dir,
                                         const std::vector<size_t> &objectives) {
    // Collect all *.gr files, sorted alphabetically for a stable ordering.
    std::vector<bfs::path> all_files;
    for (const auto &entry : bfs::directory_iterator(dir)) {
        if (entry.path().extension() == ".gr") {
            all_files.push_back(entry.path());
        }
    }
    std::sort(all_files.begin(), all_files.end());

    if (all_files.empty()) {
        std::cerr << "error: no .gr files found in " << dir << std::endl;
        exit(1);
    }

    // Select the requested subset (or all if objectives is empty).
    std::vector<bfs::path> selected;
    if (objectives.empty()) {
        selected = all_files;
    } else {
        for (size_t idx : objectives) {
            if (idx >= all_files.size()) {
                std::cerr << "error: objective index " << idx
                          << " is out of range (directory has "
                          << all_files.size() << " .gr files)" << std::endl;
                exit(1);
            }
            selected.push_back(all_files[idx]);
        }
    }

    const size_t N = selected.size();
    static const size_t BUF = 1u << 20; // 1 MiB read buffer per file
    std::vector<std::vector<char>> bufs(N, std::vector<char>(BUF));
    std::vector<std::ifstream> streams(N);
    for (size_t i = 0; i < N; i++) {
        streams[i].rdbuf()->pubsetbuf(bufs[i].data(), BUF);
        streams[i].open(selected[i].string());
        if (!streams[i]) {
            std::cerr << "error: cannot open " << selected[i] << std::endl;
            exit(1);
        }
    }

    std::vector<Edge> edges;
    size_t max_node_num = 0;
    std::vector<std::string> lines(N);

    while (true) {
        // Read one line from every stream simultaneously.
        bool any_eof = false;
        for (size_t i = 0; i < N; i++) {
            if (!std::getline(streams[i], lines[i])) {
                any_eof = true;
                break;
            }
        }
        if (any_eof) break;

        auto line0 = parse_gr_line(lines[0]);
        if (line0.type != 'a') continue; // skip c / p lines

        std::vector<size_t> costs;
        costs.reserve(N);
        costs.push_back(line0.cost);

        for (size_t i = 1; i < N; i++) {
            auto li = parse_gr_line(lines[i]);
            if (li.src != line0.src || li.tgt != line0.tgt) {
                std::cerr << "error: files are out of sync at edge "
                          << line0.src << " -> " << line0.tgt
                          << " in " << selected[i] << std::endl;
                exit(1);
            }
            costs.push_back(li.cost);
        }

        edges.emplace_back(line0.src, line0.tgt, costs);
        max_node_num = std::max({max_node_num, line0.src, line0.tgt});
    }

    return {max_node_num, edges, true};
}

// ── Public entry point ────────────────────────────────────────────────────────

AdjacencyMatrix Parser::parse_graph(const std::string &default_files_directory,
                                     const std::vector<size_t> &objectives) {
    return parse_multi_file(default_files_directory, objectives);
}
