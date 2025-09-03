"""Top-level shim module named `_rclpy` to provide compatibility on Humble.

This module imports the compiled extension available as
`rclpy._rclpy_pybind11` (Humble) or `rclpy._rclpy` (older) and re-exports it as
the top-level module `_rclpy`. Installing this via `py_modules` makes
`import _rclpy` succeed for code expecting the legacy name.
"""
import importlib
import sys
import logging

logger = logging.getLogger(__name__)

def _try_import(name):
    try:
        return importlib.import_module(name)
    except Exception:
        return None

# Try modern Humble name first
mod = _try_import('rclpy._rclpy_pybind11')
if mod is None:
    mod = _try_import('rclpy._rclpy')
if mod is None:
    mod = _try_import('_rclpy')

if mod is not None:
    # Re-export attributes on this module
    globals().update({k: getattr(mod, k) for k in dir(mod) if not k.startswith('__')})
    # Ensure sys.modules maps the name to this module object
    sys.modules.setdefault('_rclpy', sys.modules.get(__name__))
    logger.info('Installed _rclpy shim wrapping %s', getattr(mod, '__name__', '<unknown>'))
else:
    logger.warning('Could not find compiled rclpy extension to wrap; import _rclpy will fail')
