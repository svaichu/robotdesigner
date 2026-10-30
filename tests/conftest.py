"""Registers the test-only task provider (``tests/toy_provider``) under the namespace ``toy``, lazily: nothing is imported
until a test builds or parses a ``toy/...`` task."""

from robotdesigner.config import register_task_provider

register_task_provider("toy", "tests.toy_provider:PROVIDER", replace=True)
