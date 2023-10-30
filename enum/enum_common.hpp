#pragma once

#include <type_traits>
#include <utility>

#include "equality.hpp"

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

template <Enum E, E e>
    requires(!EnumHasLast<E>)
static constexpr auto EnumValue() noexcept {
	return e;
}

template <EnumHasLast E, E e>
static constexpr auto EnumValue() noexcept
    requires(NotEqual<underlying(e), underlying(decltype(e)::LAST)>)
{
	return e;
}

template <auto value>
static constexpr auto EnumValue() noexcept {
	return EnumValue<decltype(value), value>();
}

template <auto e>
static constexpr auto UEnumValue() noexcept {
	return underlying(EnumValue<decltype(e), e>());
}
/*
template <auto e>
concept EnumValueNotLast = EnumHasLast<decltype(e)> && (e != decltype(e)::LAST);

template <auto value>
    requires EnumValueNotLast<value>
struct EnumValue : public std::integral_constant<decltype(value), value> {};
*/
};  // namespace enum_utils