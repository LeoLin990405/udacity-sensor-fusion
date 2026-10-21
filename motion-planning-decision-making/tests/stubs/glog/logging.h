#pragma once

#include <cassert>

#define CHECK_EQ(left, right) assert((left) == (right))
#define CHECK_NOTNULL(pointer) assert((pointer) != nullptr)
