IMAGE ?= amr-tugger-nav:jazzy
RUN = docker run --rm --network none -v $(CURDIR)/results:/ws/results -v $(CURDIR)/scripts:/ws/scripts:ro $(IMAGE) bash -c
SETUP = . /ws/ros2_ws/install/setup.sh && cd /ws/ros2_ws

.PHONY: image test eval report all space lint map

map:
	python3 scripts/make_map.py

image:
	docker build -f docker/Dockerfile -t $(IMAGE) .

# Every GoogleTest suite and the launch_testing integration test (full stack: order in -> state finished).
test: image
	$(RUN) '$(SETUP) && colcon test --event-handlers console_direct+ ; colcon test-result --verbose ; python3 /ws/scripts/collect_tests.py /ws/results/tests.json && sh /ws/scripts/env_info.sh'

# 90 missions x 3 conditions (OFF, ON, ON with k_approach 1.6), 6 containers per condition in parallel.
eval: image
	python3 scripts/run_eval.py

report:
	python3 scripts/make_report.py

all: test eval report space

space:
	python3 scripts/build_space.py

lint:
	ruff check scripts ros2_ws/src/tugger_nav/test
