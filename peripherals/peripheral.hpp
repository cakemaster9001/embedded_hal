#pragma once

#include <concepts>
#include <type_traits>

template <typename P>
concept Peripheral = requires(P p) {
	{ P::num_regs() } -> std::same_as<std::size_t>;

	{ p.configure() };
};