/* STUB uORB::Subscription — sem atualizações (fora do escopo do driver). */
#pragma once
#include <px4_platform_common/px4_common_stub.h>
#define ORB_ID(name) 0
namespace uORB {
class Subscription { public: Subscription(int) {} bool updated() { return false; } template<class T> bool copy(T *) { return false; } };
template<class T> class SubscriptionData : public Subscription { public: SubscriptionData(int i) : Subscription(i) {} bool update() { return false; } const T &get() const { return _d; } private: T _d{}; };
}
