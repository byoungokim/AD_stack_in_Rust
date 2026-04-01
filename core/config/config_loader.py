"""YAML config loader with defaults and validation."""

import logging
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

logger = logging.getLogger(__name__)


def load_config(path: str, defaults: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Load a YAML config file and merge with defaults.

    Args:
        path: Path to the YAML config file.
        defaults: Default values to use if not specified in the file.

    Returns:
        Merged configuration dictionary.
    """
    config = dict(defaults) if defaults else {}
    config_path = Path(path)

    if not config_path.exists():
        logger.warning("Config file not found: %s — using defaults", path)
        return config

    with open(config_path) as f:
        file_config = yaml.safe_load(f) or {}

    _deep_merge(config, file_config)
    logger.info("Loaded config from %s", path)
    return config


def _deep_merge(base: dict, override: dict) -> None:
    """Recursively merge override into base (mutates base)."""
    for key, value in override.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value
