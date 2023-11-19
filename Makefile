.PHONY: test
test:
	@echo "Running tests..."
	@echo "\nRunning tests for `tput bold`communication`tput sgr0`..."
	cd communication; \
	if [ $$? -eq 0 ]; then \
		cargo test; \
		cd ..; \
	fi
	@echo "\nRunning tests for `tput bold`bluetooth-comms`tput sgr0`..."
	cd bluetooth-comms; \
	if [ $$? -eq 0 ]; then \
		cargo test; \
		cd ..; \
	fi
	@echo "\nRunning tests for `tput bold`kinematics`tput sgr0`..."
	cd kinematics; \
	if [ $$? -eq 0 ]; then \
		cargo test; \
		cd ..; \
	fi
	@echo "\nRunning tests for `tput bold`simulator`tput sgr0`..."
	cd simulator; \
	if [ $$? -eq 0 ]; then \
		cargo test; \
		cd ..; \
	fi
	@echo "\nRunning tests for `tput bold`state`tput sgr0`..."
	cd state; \
	if [ $$? -eq 0 ]; then \
		cargo test; \
		cd ..; \
	fi

.PHONY: check
check:
	@echo "\nChecking code..."
	@echo "\nChecking code for `tput bold`communication`tput sgr0`..."
	cd communication; \
	if [ $$? -eq 0 ]; then \
		cargo clippy; \
		cd ..; \
	fi
	@echo "\nChecking code for `tput bold`bluetooth-comms`tput sgr0`..."
	cd bluetooth-comms; \
	if [ $$? -eq 0 ]; then \
		cargo clippy; \
		cd ..; \
	fi
	@echo "\nChecking code for `tput bold`android-integration-testing`tput sgr0`..."
	cd android-integration-testing; \
	if [ $$? -eq 0 ]; then \
		cargo clippy; \
		cd ..; \
	fi
	@echo "\nChecking code for `tput bold`kinematics`tput sgr0`..."
	cd kinematics; \
	if [ $$? -eq 0 ]; then \
		cargo clippy; \
		cd ..; \
	fi
	@echo "\nChecking code for `tput bold`simulator`tput sgr0`..."
	cd simulator; \
	if [ $$? -eq 0 ]; then \
		cargo clippy; \
		cd ..; \
	fi
	@echo "\nChecking code for `tput bold`state`tput sgr0`..."
	cd state; \
	if [ $$? -eq 0 ]; then \
		cargo clippy; \
		cd ..; \
	fi
	@echo "\nChecking code for `tput bold`mecha-ferris`tput sgr0`..."
	cd mecha-ferris; \
	if [ $$? -eq 0 ]; then \
		cargo clippy; \
		cargo clippy --examples --features="defmt"; \
		cd ..; \
	fi
	@echo "\nChecking code for `tput bold`nrf-comms`tput sgr0`..."
	cd nrf-comms; \
	if [ $$? -eq 0 ]; then \
		cargo clippy; \
		cd ..; \
	fi

.PHONY: clean
clean:
	@echo "\Cleaning code..."
	@echo "\Cleaning code for `tput bold`communication`tput sgr0`..."
	cd communication; \
	if [ $$? -eq 0 ]; then \
		cargo clean; \
		cd ..; \
	fi
	@echo "\Cleaning code for `tput bold`bluetooth-comms`tput sgr0`..."
	cd bluetooth-comms; \
	if [ $$? -eq 0 ]; then \
		cargo clean; \
		cd ..; \
	fi
	@echo "\Cleaning code for `tput bold`android-integration-testing`tput sgr0`..."
	cd android-integration-testing; \
	if [ $$? -eq 0 ]; then \
		cargo clean; \
		cd ..; \
	fi
	@echo "\Cleaning code for `tput bold`kinematics`tput sgr0`..."
	cd kinematics; \
	if [ $$? -eq 0 ]; then \
		cargo clean; \
		cd ..; \
	fi
	@echo "\Cleaning code for `tput bold`simulator`tput sgr0`..."
	cd simulator; \
	if [ $$? -eq 0 ]; then \
		cargo clean; \
		cd ..; \
	fi
	@echo "\Cleaning code for `tput bold`state`tput sgr0`..."
	cd state; \
	if [ $$? -eq 0 ]; then \
		cargo clean; \
		cd ..; \
	fi
	@echo "\Cleaning code for `tput bold`mecha-ferris`tput sgr0`..."
	cd mecha-ferris; \
	if [ $$? -eq 0 ]; then \
		cargo clean; \
		cd ..; \
	fi
	@echo "\Cleaning code for `tput bold`nrf-comms`tput sgr0`..."
	cd nrf-comms; \
	if [ $$? -eq 0 ]; then \
		cargo clean; \
		cd ..; \
	fi
