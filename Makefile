.PHONY: help install install-dev lint clean test

help:
	@echo "Teleoperate Workspace Commands"
	@echo ""
	@echo "  make install      - Install both packages"
	@echo "  make install-dev  - Install packages + dev tools"
	@echo "  make lint         - Run linter on source code"
	@echo "  make clean        - Remove build artifacts"
	@echo "  make test         - Run tests (if available)"

install:
	pip install -e src/lerobot_robot_ros
	pip install -e src/lerobot_teleoperator_devices

install-dev:
	pip install ruff pytest
	$(MAKE) install

lint:
	ruff check src/

clean:
	find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name ".pytest_cache" -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name ".ruff_cache" -exec rm -rf {} + 2>/dev/null || true

test:
	pytest src/ -v
