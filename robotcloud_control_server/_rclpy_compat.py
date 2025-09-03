"""Compatibility shim for rclpy compiled extension on Humble.

Some rclpy code expects to import a C extension named `_rclpy`. In Humble the
compiled module is named `_rclpy_pybind11`. Import that and insert it into
sys.modules under the name `_rclpy` so legacy imports succeed.
"""
import sys
import importlib
import logging

try:
    # Try the modern name used in Humble
    mod = importlib.import_module("rclpy._rclpy_pybind11")
    logging.getLogger(__name__).info("Imported rclpy._rclpy_pybind11 as compatibility shim")
    sys.modules.setdefault("_rclpy", mod)
except Exception:
    # If that fails, try other common names and finally do nothing
    try:
        mod2 = importlib.import_module("rclpy._rclpy")
        sys.modules.setdefault("_rclpy", mod2)
    except Exception:
        try:
            # If there is a top-level _rclpy extension available, expose it
            mod3 = importlib.import_module("_rclpy")
            sys.modules.setdefault("_rclpy", mod3)
        except Exception:
            # Last resort: no compiled extension visible; rclpy will raise later if used
            logging.getLogger(__name__).warning("No compiled _rclpy extension found; rclpy may fail to initialize")
"""Compatibility shim to expose compiled rclpy extension under legacy name
_rclpy for ROS 2 distributions (Humble renamed the extension to _rclpy_pybind11).

This module attempts to import the native extension under several names and
ensures sys.modules['_rclpy'] refers to the loaded extension so older importers
that expect `_rclpy` succeed.
"""
import importlib
import sys
import os

def ensure_rclpy_extension():
    # If already available, nothing to do
    try:
        import _rclpy  # noqa: F401
        return
    except Exception:
        pass

    # Try common pybind11 name inside rclpy package
    try:
        # import rclpy package to find compiled modules
        import rclpy as _rclpy_pkg
        pkgdir = os.path.dirname(_rclpy_pkg.__file__)
        # Prefer the explicit pybind11 named module if present
        candidates = []
        for fn in os.listdir(pkgdir):
            if fn.startswith('_rclpy') and fn.endswith('.so'):
                # module name inside package is rclpy.<modname>
                modname = fn[:-3]
                candidates.append(modname)
        # Prefer the pybind11 variant
        preferred = None
        for c in candidates:
            if 'pybind11' in c:
                preferred = c
                break
        if preferred is None and candidates:
            preferred = candidates[0]

        if preferred is not None:
            full = f"rclpy.{preferred}"
            mod = importlib.import_module(full)
            # Expose under legacy top-level name
            sys.modules['_rclpy'] = mod
            return
    except Exception:
        pass

    # As a last resort, try to import aerial names directly
    for name in ('_rclpy_pybind11', '_rclpy'):
        try:
            mod = importlib.import_module(name)
            sys.modules['_rclpy'] = mod
            return
        except Exception:
            continue

    # If no extension could be found, leave it to normal import errors later.


ensure_rclpy_extension()
