"""Task providers shipped with robotdesigner, one subpackage per namespace, and the contract of a task.

Importing this package imports no engine. The subpackages are discovered by ``config.providers``; nothing here lists
them. A new provider is a new subpackage that exposes ``PROVIDER`` (or a separately installed package that declares the
entry-point group ``robotdesigner.task_providers``). See ``tasks/base.py`` for what a task implements.
"""

from .base import BaseTask, TaskImpl

__all__ = ["BaseTask", "TaskImpl"]
