"""Validation errors that carry the path of the offending field."""

from __future__ import annotations


class ConfigError(ValueError):
    """Invalid configuration. The message starts with the path of the offending field."""

    def __init__(self, path: str, message: str):
        self.path = path
        self.reason = message
        super().__init__(f"{path}: {message}" if path else message)

    def prefixed(self, prefix: str) -> "ConfigError":
        """The same error, one level further from the root."""
        if not self.path:
            path = prefix
        elif self.path.startswith("["):
            path = prefix + self.path
        else:
            path = f"{prefix}.{self.path}"
        return ConfigError(path, self.reason)
