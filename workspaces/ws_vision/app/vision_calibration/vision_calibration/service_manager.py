"""
Service manager for PapJia Camera Calibration Service.

This module manages the lifecycle of all service components including
ZeroMQ server, REST API server, and provides unified control.
"""

import logging
import signal
import sys
import time
import threading
from typing import Optional, List
from concurrent.futures import ThreadPoolExecutor
import asyncio
import uvicorn

from .config import ServiceConfig, load_config
from .zmq_server import ZMQCalibrationServer
from .rest_api import CalibrationAPI


class CalibrationServiceManager:
    """
    Manager for all camera calibration service components.

    Coordinates the startup, shutdown, and monitoring of ZeroMQ and REST API servers.
    """

    def __init__(self, config: ServiceConfig):
        """
        Initialize service manager.

        Args:
            config: Service configuration
        """
        self.config = config
        self.logger = logging.getLogger(__name__)

        # Service components
        self.zmq_server = None
        self.rest_api = None
        self.uvicorn_server = None

        # Control flags
        self.running = False
        self.shutdown_event = threading.Event()

        # Thread management
        self.executor = ThreadPoolExecutor(max_workers=4)
        self.service_threads = []

        # Setup signal handlers
        self._setup_signal_handlers()

    def _setup_signal_handlers(self):
        """Setup signal handlers for graceful shutdown."""

        def signal_handler(signum, frame):
            self.logger.info(f"Received signal {signum}, initiating shutdown...")
            self.stop()

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

    def _initialize_components(self):
        """Initialize service components."""
        self.logger.info("Initializing service components...")

        # Initialize ZeroMQ server if enabled
        if self.config.zeromq.enabled:
            try:
                self.zmq_server = ZMQCalibrationServer(self.config)
                self.logger.info("ZeroMQ server initialized")
            except Exception as e:
                self.logger.error(f"Failed to initialize ZeroMQ server: {e}")
                raise

        # Initialize REST API if enabled
        if self.config.rest_api.enabled:
            try:
                self.rest_api = CalibrationAPI(self.config)

                # Create Uvicorn server config
                uvicorn_config = uvicorn.Config(
                    app=self.rest_api.app,
                    host=self.config.rest_api.host,
                    port=self.config.rest_api.port,
                    workers=1,  # Single worker for embedded mode
                    log_level=self.config.logging.level.lower(),
                    access_log=True,
                )
                self.uvicorn_server = uvicorn.Server(uvicorn_config)
                self.logger.info("REST API server initialized")
            except Exception as e:
                self.logger.error(f"Failed to initialize REST API server: {e}")
                raise

    def _start_zmq_server(self):
        """Start ZeroMQ server in a separate thread."""
        if self.zmq_server:
            try:
                self.logger.info("Starting ZeroMQ server...")
                self.zmq_server.start()
            except Exception as e:
                self.logger.error(f"ZeroMQ server failed: {e}")
                self.shutdown_event.set()

    def _start_rest_api(self):
        """Start REST API server in a separate thread."""
        if self.uvicorn_server:
            try:
                self.logger.info("Starting REST API server...")
                # Run the asyncio event loop for uvicorn
                asyncio.run(self.uvicorn_server.serve())
            except Exception as e:
                self.logger.error(f"REST API server failed: {e}")
                self.shutdown_event.set()

    def start(self):
        """Start all enabled services."""
        if self.running:
            self.logger.warning("Service manager is already running")
            return

        self.logger.info("Starting PapJia Camera Calibration Service...")

        try:
            # Initialize components
            self._initialize_components()

            self.running = True

            # Start ZeroMQ server if enabled
            if self.config.zeromq.enabled and self.zmq_server:
                zmq_thread = threading.Thread(target=self._start_zmq_server, name="ZMQServer", daemon=False)
                zmq_thread.start()
                self.service_threads.append(zmq_thread)
                self.logger.info(f"ZeroMQ server thread started on {self.config.zeromq.get_bind_address()}")

            # Start REST API server if enabled
            if self.config.rest_api.enabled and self.uvicorn_server:
                api_thread = threading.Thread(target=self._start_rest_api, name="RestAPI", daemon=False)
                api_thread.start()
                self.service_threads.append(api_thread)
                self.logger.info(f"REST API server thread started on {self.config.rest_api.get_bind_address()}")

            # Check if any services are enabled
            if not self.service_threads:
                self.logger.warning("No services are enabled in configuration")
                return

            self.logger.info("All services started successfully")

            # Wait for shutdown signal or service failure
            self._wait_for_shutdown()

        except Exception as e:
            self.logger.error(f"Failed to start services: {e}")
            self.stop()
            raise

    def _wait_for_shutdown(self):
        """Wait for shutdown signal or service failure."""
        self.logger.info("Service manager is running. Press Ctrl+C to stop.")

        try:
            while self.running and not self.shutdown_event.is_set():
                # Check if all service threads are still alive
                alive_threads = [t for t in self.service_threads if t.is_alive()]

                if len(alive_threads) < len(self.service_threads):
                    # Some service thread has died
                    self.logger.error("One or more service threads have stopped unexpectedly")
                    self.shutdown_event.set()
                    break

                # Sleep for a short interval
                time.sleep(1)

        except KeyboardInterrupt:
            self.logger.info("Received keyboard interrupt")
            self.shutdown_event.set()

    def stop(self):
        """Stop all services gracefully."""
        if not self.running:
            return

        self.logger.info("Stopping PapJia Camera Calibration Service...")

        self.running = False
        self.shutdown_event.set()

        # Stop ZeroMQ server
        if self.zmq_server:
            try:
                self.zmq_server.stop()
                self.logger.info("ZeroMQ server stopped")
            except Exception as e:
                self.logger.error(f"Error stopping ZeroMQ server: {e}")

        # Stop REST API server
        if self.uvicorn_server:
            try:
                self.uvicorn_server.should_exit = True
                self.logger.info("REST API server shutdown initiated")
            except Exception as e:
                self.logger.error(f"Error stopping REST API server: {e}")

        # Wait for service threads to finish
        for thread in self.service_threads:
            if thread.is_alive():
                self.logger.info(f"Waiting for {thread.name} to stop...")
                thread.join(timeout=10)
                if thread.is_alive():
                    self.logger.warning(f"{thread.name} did not stop gracefully")

        # Shutdown thread pool
        self.executor.shutdown(wait=True)

        self.logger.info("Service manager stopped")

    def restart(self):
        """Restart all services."""
        self.logger.info("Restarting services...")
        self.stop()
        time.sleep(2)  # Brief pause
        self.start()

    def get_status(self) -> dict:
        """Get status of all service components."""
        status = {
            "service_manager": {"running": self.running, "shutdown_event_set": self.shutdown_event.is_set()},
            "zeromq_server": {
                "enabled": self.config.zeromq.enabled,
                "running": self.zmq_server.running if self.zmq_server else False,
                "address": self.config.zeromq.get_bind_address() if self.config.zeromq.enabled else None,
                "calibration_config_file": self.config.zeromq.calibration_config_file if self.config.zeromq.enabled else None,
            },
            "rest_api": {
                "enabled": self.config.rest_api.enabled,
                "running": bool(self.uvicorn_server and not self.uvicorn_server.should_exit),
                "address": (
                    f"http://{self.config.rest_api.get_bind_address()}" if self.config.rest_api.enabled else None
                ),
                "calibration_config_file": self.config.rest_api.calibration_config_file if self.config.rest_api.enabled else None,
            },
            "threads": [
                {"name": thread.name, "alive": thread.is_alive(), "daemon": thread.daemon}
                for thread in self.service_threads
            ],
        }
        return status

    def health_check(self) -> dict:
        """Perform health check on all components."""
        health = {"overall_status": "healthy", "timestamp": time.time(), "components": {}}

        # Check ZeroMQ server
        if self.config.zeromq.enabled:
            if self.zmq_server and self.zmq_server.running:
                health["components"]["zeromq"] = {"status": "healthy", "running": True}
            else:
                health["components"]["zeromq"] = {"status": "unhealthy", "running": False}
                health["overall_status"] = "unhealthy"

        # Check REST API
        if self.config.rest_api.enabled:
            if self.uvicorn_server and not self.uvicorn_server.should_exit:
                health["components"]["rest_api"] = {"status": "healthy", "running": True}
            else:
                health["components"]["rest_api"] = {"status": "unhealthy", "running": False}
                health["overall_status"] = "unhealthy"

        return health


def create_service_manager(config_path: Optional[str] = None) -> CalibrationServiceManager:
    """
    Factory function to create a service manager.

    Args:
        config_path: Path to configuration file

    Returns:
        CalibrationServiceManager instance
    """
    config = load_config(config_path)
    return CalibrationServiceManager(config)


def run_service(config_path: Optional[str] = None):
    """
    Run the complete camera calibration service.

    Args:
        config_path: Path to configuration file
    """
    manager = create_service_manager(config_path)

    try:
        manager.start()
    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt")
    except Exception as e:
        print(f"Service failed: {e}")
        sys.exit(1)
    finally:
        manager.stop()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Run PapJia Camera Calibration Service")
    parser.add_argument("--config", "-c", help="Path to configuration file")
    parser.add_argument("--status", action="store_true", help="Show service status and exit")

    args = parser.parse_args()

    if args.status:
        # Show status of running service
        manager = create_service_manager(args.config)
        status = manager.get_status()
        import json

        print(json.dumps(status, indent=2))
    else:
        # Run the service
        run_service(args.config)
