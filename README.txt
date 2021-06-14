Dependencies:
	ros (Robotics Operatin System, any version)
	Rigid Body Dynamics Library (rbdl)
	Open Motion Planning Library (ompl)
	

Installation Instructions:
1. Open a Terminal
2. Install ROS:
	Just follow these instructions here. Do a desktop-full installation. This will take a while.
	http://wiki.ros.org/noetic/Installation/Ubuntu
	

3. Install RBDL
	run the following command:
	git clone https://github.com/rbdl/rbdl.git

	This downloads the repository to the folder RBDL
	Then enter the following commands:

	cd rbdl
	mkdir build
	cd build
	cmake -D CMAKE_BUILD_TYPE=Release ../
	make
	sudo make install
	
	That should install the rigid body dynamics library. Let me know if there are any errors.

3. Install OMPL
	Just run this one command:
	sudo apt install ros-noetic-ompl

Thats all. If you have errors you can usually solve it by googling it, but if
that doesn't work let me know and I can try to help.
