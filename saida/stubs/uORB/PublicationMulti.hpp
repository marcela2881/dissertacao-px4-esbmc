/* STUB uORB::PublicationMulti — guarda a última mensagem publicada para o harness checar. */
#pragma once
#include <uORB/Publication.hpp>
namespace uORB {
template<class T> class PublicationMulti : public Publication<T> {
public:
	PublicationMulti(int id) : Publication<T>(id) {}
	bool advertise() { return true; }
	void unadvertise() {}
};
}
