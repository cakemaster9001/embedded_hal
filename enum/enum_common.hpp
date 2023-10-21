#pragma once

#include <type_traits>
#include <utility>

namespace enum_utils {
template <typename E>
concept Enum = std::is_enum_v<E>;

template <typename E>
concept EnumHasLast = Enum<E> && requires { E::LAST; };

template <Enum E>
using UnderlyingT =
    typename std::make_unsigned_t<typename std::underlying_type_t<E>>;

template <Enum E>
static constexpr UnderlyingT<E> underlying(E e) {
	return static_cast<UnderlyingT<E>>(e);
}
};  // namespace enum_utils