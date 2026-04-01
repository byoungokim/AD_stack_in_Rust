.PHONY: all build build-cpp proto clean test test-unit test-integration

BUILD_DIR := build

all: build

# Build everything: protobuf + C++ + Python proto
build: proto build-cpp

# Generate Python protobuf files
proto:
	@echo "==> Generating Protobuf (Python)..."
	@mkdir -p proto/gen_py
	protoc --proto_path=proto --python_out=proto/gen_py proto/*.proto
	@echo "==> Protobuf Python generation done."

# Build C++ (includes protobuf C++ generation via CMake)
build-cpp:
	@echo "==> Building C++..."
	@mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. && make -j$$(nproc)
	@echo "==> C++ build done."

clean:
	rm -rf $(BUILD_DIR)
	rm -rf proto/gen_py

# Run all tests
test: test-unit test-integration

test-unit:
	@echo "==> Running unit tests..."
	cd $(BUILD_DIR) && ctest --output-on-failure
	python -m pytest tests/unit/ -v

test-integration:
	@echo "==> Running integration tests..."
	python -m pytest tests/integration/ -v

# Launch the system
run:
	python tools/launcher.py config/system.yaml

run-sim:
	python tools/launcher.py config/system.yaml --sim

run-e2e:
	python tools/launcher.py config/system.yaml --mode e2e
