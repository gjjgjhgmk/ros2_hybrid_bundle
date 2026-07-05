"""
Main entry point for PapJia Camera Calibration Service.

This module provides the main entry point for running the camera calibration service
with both ZeroMQ and REST API interfaces.
"""

import argparse
import logging
import sys
from typing import Optional

from .service_manager import run_service, create_service_manager
from .config import load_config, save_config


def main():
    """Main entry point for the camera calibration service."""
    parser = argparse.ArgumentParser(
        description="PapJia Camera Calibration Service",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
            Examples:
            # Run with default configuration
            python -m vision_calibration.main

            # Run with custom configuration file
            python -m vision_calibration.main --config config.yaml

            # Show service status
            python -m vision_calibration.main --status

            # Generate default configuration file
            python -m vision_calibration.main --generate-config config.yaml

            # Run only ZeroMQ server
            python -m vision_calibration.main --zmq-only

            # Run only REST API server
            python -m vision_calibration.main --rest-only
        """,
    )

    parser.add_argument("--config", "-c", help="Path to configuration file")

    parser.add_argument("--status", action="store_true", help="Show service status and exit")

    parser.add_argument("--generate-config", metavar="CONFIG_PATH", help="Generate default configuration file and exit")

    parser.add_argument("--zmq-only", action="store_true", help="Run only ZeroMQ server")

    parser.add_argument("--rest-only", action="store_true", help="Run only REST API server")

    parser.add_argument(
        "--log-level",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        default="INFO",
        help="Logging level (default: INFO)",
    )

    args = parser.parse_args()

    # Handle generate-config option
    if args.generate_config:
        try:
            config = load_config()
            save_config(config, args.generate_config)
            print(f"Default configuration saved to: {args.generate_config}")
            return
        except Exception as e:
            print(f"Failed to generate configuration: {e}")
            sys.exit(1)

    # Handle status option
    if args.status:
        try:
            manager = create_service_manager(args.config)
            status = manager.get_status()
            import json

            print(json.dumps(status, indent=2))
            return
        except Exception as e:
            print(f"Failed to get service status: {e}")
            sys.exit(1)

    # Load configuration
    config = load_config(args.config)

    # Override configuration based on command line options
    if args.zmq_only:
        config.rest_api.enabled = False
        print("Running ZeroMQ server only")
    elif args.rest_only:
        config.zeromq.enabled = False
        print("Running REST API server only")

    # Set log level
    config.logging.level = args.log_level

    # Set up logging
    logging.basicConfig(level=getattr(logging, config.logging.level), format=config.logging.format)

    # Run the service
    try:
        run_service(config)
    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt")
    except Exception as e:
        print(f"Service failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
