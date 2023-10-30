#include <iostream>

#include "enum_test.hpp"
#include "enum_utils.hpp"

using namespace enum_utils;
enum class Test { Elem0, Elem1, Elem2, Elem3, Elem4, LAST };

enum class Test2 { Elem0, Elem1, LAST };

using TestFlag = EnumFlag<Test>;
using TestArray = EnumArray<Test, 8>;

void testEnum() {
	TestArray testArr{};

	testArr[0] = Test::Elem3;
	testArr[2] = Test::Elem4;

	std::cout << testArr << "\n";

	TestFlag testFlag{};

	std::cout << testFlag << "\n";
	testFlag |= Test::Elem2;
	testFlag |= Test::Elem0;

	testFlag.set<Test::Elem4>();

	std::cout << testFlag << "\n";
}