# ----------------------------- DEFAULT ----------------------------- #

.ONESHELL:
.DEFAULT_GOAL := make

make:
	make setup
	make run

# --------------------------- SETUP & RUN --------------------------- #

setup:
	echo "---------------- Installing ----------------"
	cd ~/ros2_ws/src/zenoh-pointcloud/
	./install_zenoh.sh

run:
	echo "---------------- Executing -----------------"
	cd ~/ros2_ws/src/zenoh-pointcloud/init/
	./zenoh_pointcloud_rosject.sh
