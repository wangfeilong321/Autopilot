#include <Controller.h>

using namespace std;

int main(Platform::Array<Platform::String^>^ args) {
	try {
		Controller C;
		C.Connect();
		if (!C.Connected())
			throw("Unexpected exception during connection procedures!");
		while (C.Run());
	}
	catch (exception& e) {
		string what = e.what();
	}
	return EXIT_SUCCESS;
}