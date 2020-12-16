#include <string>
#include <vector>
#include "ros_interface.hpp"
#include "movement_primitives.hpp"

int main(int argc, char **argv) {
	init(argc, argv);
	// main loop
	while (isOk()) {
		rotate(90,1);
		processCallbacks();
		if(!timeStep())
			break;
	}
	quit(0);
	return 0;
}
