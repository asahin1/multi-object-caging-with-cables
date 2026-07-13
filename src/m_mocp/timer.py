# Standard
from functools import wraps
import time

class GlobalTimeoutError(Exception):
    """Custom type to catch timeout errors."""
    pass


class Timer:
    def __init__(self, timeout_seconds) -> None:
        self.start_time = time.perf_counter()
        self.timeout = timeout_seconds

    @property
    def elapsed(self) -> float:
        return time.perf_counter() - self.start_time

    @property
    def remaining(self) -> float:
        return max(0.0, self.timeout - self.elapsed)

    def check(self):
        if self.elapsed > self.timeout:
            raise GlobalTimeoutError(f"Timeout after {self.elapsed:.3f}s")


def enforce_timeout(func):
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        self.timer.check()
        return func(self, *args, **kwargs)

    return wrapper
