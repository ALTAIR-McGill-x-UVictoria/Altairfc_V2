from __future__ import annotations

import logging
import sys


# ANSI escape codes
_RESET  = "\033[0m"
_BOLD   = "\033[1m"
_DIM    = "\033[2m"

_GREY   = "\033[38;5;245m"
_CYAN   = "\033[38;5;51m"
_GREEN  = "\033[38;5;82m"
_YELLOW = "\033[38;5;220m"
_ORANGE = "\033[38;5;208m"
_RED    = "\033[38;5;196m"
_PINK   = "\033[38;5;213m"

# Per-level styling: (level_color, message_color)
_LEVEL_STYLES: dict[int, tuple[str, str]] = {
    logging.DEBUG:    (_DIM + _GREY,          _DIM + _GREY),
    logging.INFO:     (_BOLD + _CYAN,         _RESET),
    logging.WARNING:  (_BOLD + _YELLOW,       _YELLOW),
    logging.ERROR:    (_BOLD + _ORANGE,       _ORANGE),
    logging.CRITICAL: (_BOLD + _RED,          _BOLD + _RED),
}


class ColorFormatter(logging.Formatter):
    """
    Logging formatter that adds ANSI color to terminal output.

    Color scheme:
        DEBUG    — dim grey       (verbose noise, visually recessed)
        INFO     — bold cyan      (normal status messages)
        WARNING  — yellow         (non-fatal issues, e.g. retry)
        ERROR    — orange         (task failures, recoverable errors)
        CRITICAL — bold red       (fatal, system shutdown)

    Colors are automatically disabled when stdout is not a TTY
    (e.g. piped to a file or journald), keeping logs clean in that case.
    """

    _BASE_FMT  = "%(asctime)s  %(levelname)-8s  %(name)s  %(message)s"
    _DATE_FMT  = "%Y-%m-%dT%H:%M:%S"

    def __init__(self, use_color: bool = True) -> None:
        super().__init__(fmt=self._BASE_FMT, datefmt=self._DATE_FMT)
        self.use_color = use_color

    def format(self, record: logging.LogRecord) -> str:
        if not self.use_color:
            return super().format(record)

        level_color, msg_color = _LEVEL_STYLES.get(
            record.levelno, (_RESET, _RESET)
        )

        # Color the level name
        record.levelname = f"{level_color}{record.levelname:<8}{_RESET}"

        # Color the logger name (module path)
        record.name = f"{_DIM}{_PINK}{record.name}{_RESET}"

        # Color the timestamp
        formatted = super().format(record)

        # Re-inject color around the message portion
        # Replace plain message with colored version
        plain_msg = record.getMessage()
        colored_msg = f"{msg_color}{plain_msg}{_RESET}"
        formatted = formatted.replace(plain_msg, colored_msg, 1)

        # Color the timestamp prefix (first 19 chars: YYYY-MM-DDTHH:MM:SS)
        formatted = f"{_DIM}{_GREY}{formatted[:19]}{_RESET}{formatted[19:]}"

        return formatted


def setup_logging(level: str = "INFO") -> None:
    """
    Configure the root logger with color output if stdout is a TTY,
    plain output otherwise. Call once at process startup in main.py.
    """
    use_color = hasattr(sys.stdout, "isatty") and sys.stdout.isatty()
    formatter = ColorFormatter(use_color=use_color)

    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(formatter)

    root = logging.getLogger()
    root.handlers.clear()
    root.addHandler(handler)
    root.setLevel(level)
