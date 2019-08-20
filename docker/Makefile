SHELL := /bin/bash

base:
	docker build -t aquadrone_latest -t aquadrone_base -f DockerfileBase ..

nvidia:
	docker build -t aquadrone_latest -t aquadrone_nvidia -f DockerfileNvidia --build-arg BASE_IMAGE=aquadrone_base ..