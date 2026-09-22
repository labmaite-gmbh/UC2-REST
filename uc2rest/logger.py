try:
    import loguru as _loguru
except ImportError:  # uc2rest is also used standalone, without loguru
    _loguru = None


class Logger(object):
    """uc2rest's internal logger. Forwards to loguru when it is available so
    serial-layer messages ('Wrong Firmware.', 'Serial command timed out ...',
    'Trying out port ... failed') land in the same file sinks as the rest of
    the process -- they were print()-only and therefore missing from the
    2026-09-21 incident's logs. depth=1 makes the record point at the
    mserial.py call site, not at this wrapper."""

    def __init__(self):
        pass

    def error(self, message):
        if _loguru is not None:
            _loguru.logger.opt(depth=1).error(str(message))
        else:
            print(message)

    def warning(self, message):
        if _loguru is not None:
            _loguru.logger.opt(depth=1).warning(str(message))
        else:
            print(message)

    def debug(self, message):
        if _loguru is not None:
            _loguru.logger.opt(depth=1).debug(str(message))
        else:
            print(message)