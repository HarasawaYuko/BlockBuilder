#include "DEBUG.h"

Debug* Debug::instance() {
	static Debug instance;
	return &instance;
}