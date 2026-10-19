/* STUB uORB::Publication — guarda a última mensagem publicada para o harness checar. */
#pragma once
#include <px4_platform_common/px4_common_stub.h>
#ifndef ORB_ID
#define ORB_ID(name) 0
#endif
namespace uORB {
template<class T> class Publication {
public:
	Publication(int) {}
	bool publish(const T &d) { last = d; published = true; return true; }
	T last{}; bool published{false};
};
}
