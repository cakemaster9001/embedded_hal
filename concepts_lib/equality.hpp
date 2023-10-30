#pragma once

#include <concepts>
#include <type_traits>

template <std::size_t num, std::size_t other>
concept Equal = (num == other);
template <std::size_t num, std::size_t other>
concept NotEqual = !Equal<num, other>;

template <std::size_t num, std::size_t lower_bound>
concept GreaterThan = (num > lower_bound);

template <std::size_t num, std::size_t lower_bound>
concept GreaterThanEqual =
    GreaterThan<num, lower_bound> || Equal<num, lower_bound>;

template <std::size_t num, std::size_t upper_bound>
concept LessThan = !GreaterThanEqual<num, upper_bound>;
template <std::size_t num, std::size_t upper_bound>
concept LessThanEqual = !GreaterThan<num, upper_bound>;
