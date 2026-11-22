SHELL := /bin/bash
export PYTHONDONTWRITEBYTECODE := 1
.DEFAULT_GOAL := help

PYTHON ?= python3
ROS_DISTRO ?= humble
SMOKE_TIMEOUT ?= 600
SMOKE_ARGS ?=
RUN_ARGS ?=

.PHONY: help setup build run run-headless run-optical-flow run-mapping run-localized doctor lint test test-ros test-all check smoke smoke-mapping model release

help: ## Show development commands
	@awk 'BEGIN {FS = ":.*## "} /^[a-z-]+:.*## / {printf "  %-12s %s\n", $$1, $$2}' $(MAKEFILE_LIST)

setup: ## Install Ubuntu 22.04 dependencies and build the full simulation
	bash tools/setup_ubuntu_22_04.sh

build: ## Build the ROS workspace
	bash tools/build_workspace.sh

run: ## Start Gazebo, RViz and the automatic competition mission
	bash start.sh $(RUN_ARGS)

run-headless: ## Start the simulation without GUI windows
	bash start.sh --headless $(RUN_ARGS)

run-optical-flow: ## Start the simulation using optical-flow ExternalNav
	bash start.sh --optical-flow $(RUN_ARGS)

run-mapping: ## Build a 3D point-cloud map while flying the preset route
	bash start.sh --mapping $(RUN_ARGS)

run-localized: ## Fly the preset route using a saved 3D map
	bash start.sh --localize $(RUN_ARGS)

doctor: ## Check the runtime environment and launch file without starting simulation
	bash start.sh --check $(RUN_ARGS)

lint: ## Check Python style
	$(PYTHON) -m flake8 src tests tools

test: ## Run standalone tests without ROS, Gazebo or SITL
	PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 $(PYTHON) -m pytest -p no:cacheprovider -q tests/unit

test-ros: ## Run ROS message and node regression tests
	@set -eo pipefail; source tools/workspace_env.sh; source /opt/ros/$(ROS_DISTRO)/setup.bash; $(PYTHON) -m pytest -p no:cacheprovider -q tests/ros

test-all: ## Run standalone and ROS tests
	@set -eo pipefail; source tools/workspace_env.sh; source /opt/ros/$(ROS_DISTRO)/setup.bash; $(PYTHON) -m pytest -p no:cacheprovider -q

check: lint test ## Check publishable files, model reproducibility and configured scene dimensions
	$(PYTHON) tools/check_open_source_hygiene.py
	$(PYTHON) tools/generate_competition_x500.py
	$(PYTHON) tools/validate_rule_dimensions.py

smoke: ## Run the truth reference mission (set SMOKE_VISUAL=1 for GUI; build first)
	bash tools/run_headless_smoke.sh $(SMOKE_TIMEOUT) $(SMOKE_ARGS)

smoke-mapping: ## Verify mapping, map reload and localization in two complete missions
	bash tools/run_mapping_smoke.sh $(SMOKE_TIMEOUT) $(SMOKE_ARGS)

model: ## Regenerate the competition x500 model after editing its template
	$(PYTHON) tools/generate_competition_x500.py --write

release: ## Audit staged content and export a simulation-only source ZIP outside the repository
	@set -e; source tools/workspace_env.sh; $(PYTHON) tools/package_release.py --output "$$COMPETITION_LOCAL_DIR/releases/aeromind-sim-source.zip"
