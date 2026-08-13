#pragma once

#include "TabulatedFunction.h"
#include <array>
#include <cstddef>

namespace capicola {

template <typename T, size_t n>
class Tabulator
{
public:
    Tabulator() {}

    template <Mirror M = Mirror::None,
              Extend E = Extend::Clamp,
              typename Func>
    TabulatedFunction<T, n, M, E> tabulate(Func function, T min, T max)
    {
        std::array<T, n> table{};
        T increment = (max - min) / static_cast<T>(n - 1);
        T val       = min;
        for (auto& cell : table) {
            cell = function(val);
            val += increment;
        }
        return TabulatedFunction<T, n, M, E>{table, min, max};
    }
};

} // namespace capicola
