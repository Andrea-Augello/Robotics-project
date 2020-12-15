#include <string>
#include <vector>
#include "ros_interface.hpp"

int main(int argc, char **argv) {
	init(argc, argv);
	// main loop
	while (isOk()) {
		if(!timeStep())
			break;
	}
	quit(0);
	return 0;
}
