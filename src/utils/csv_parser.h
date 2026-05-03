/**
 * @file csv_parser.h
 * @brief Simple CSV parser for true_alphabet_modes.csv.
 */

#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "../core/physics.h"

namespace chladni {

struct BatchRow {
    int mode_id;
    double frequency;
    double quality;
    ::std::vector<Transducer> layout;
};

class CSVParser {
public:
    static ::std::vector<BatchRow> parse_batch_csv(const ::std::string& path) {
        ::std::vector<BatchRow> rows;
        ::std::ifstream file(path);
        if (!file.is_open()) return rows;

        ::std::string line;
        if (!::std::getline(file, line)) return rows;

        while (::std::getline(file, line)) {
            ::std::stringstream ss(line);
            ::std::string item;
            BatchRow row;
            
            if (!::std::getline(ss, item, ',')) break;
            try { row.mode_id = ::std::stoi(item); } catch (...) { continue; }
            
            if (!::std::getline(ss, item, ',')) break;
            try { row.frequency = ::std::stod(item); } catch (...) { continue; }
            
            if (!::std::getline(ss, item, ',')) break;
            try { row.quality = ::std::stod(item); } catch (...) { continue; }

            while (::std::getline(ss, item, ',')) {
                Transducer t;
                try {
                    t.x = ::std::stod(item);
                    if (!::std::getline(ss, item, ',')) break;
                    t.y = ::std::stod(item);
                    if (!::std::getline(ss, item, ',')) break;
                    t.phase_rad = ::std::stod(item) * 3.14159 / 180.0;
                    if (!::std::getline(ss, item, ',')) break;
                    t.amplitude = ::std::stod(item);
                    t.frequency = ::std::nullopt;
                    row.layout.push_back(t);
                } catch (...) { break; }
            }
            rows.push_back(row);
        }
        return rows;
    }
};

} // namespace chladni
