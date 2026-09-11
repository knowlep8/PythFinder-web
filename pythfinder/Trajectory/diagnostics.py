"""Problems found while building a run, as data instead of printed text.

The builder used to print its complaints and carry on: a marker placed past the
end of a move was silently dropped, with a note on a terminal nobody was
looking at. In a browser there is no terminal at all, and the person planning
the run is eleven years old -- the problem has to appear next to the step that
caused it.

So every complaint becomes a Diagnostic: which step, what happened, and what to
do about it. The simulator still prints them, so nothing is lost on the desktop.

See docs/web-planner.md, step 1.6.
"""


class Diagnostic():
    WARNING = "warning"   # the run still works, but not as asked
    ERROR = "error"       # the run cannot be used as it stands

    def __init__(self,
                 level: str,
                 message: str,
                 step: int | None = None,
                 suggestion: str | None = None,
                 time_ms: int | None = None):
        """
        Args:
            level: WARNING or ERROR.
            message: what happened, in words a team member would understand.
            step: which step of the run, counted from 0, or None for the run
                as a whole. Displayed as "step 1" for index 0.
            suggestion: what to do about it, when there is an obvious answer.
            time_ms: when in the run it happens, where that makes sense.
        """

        self.level = level
        self.message = message
        self.step = step
        self.suggestion = suggestion
        self.time_ms = time_ms

    @classmethod
    def warning(cls, message, step = None, suggestion = None, time_ms = None):
        return cls(cls.WARNING, message, step, suggestion, time_ms)

    @classmethod
    def error(cls, message, step = None, suggestion = None, time_ms = None):
        return cls(cls.ERROR, message, step, suggestion, time_ms)

    def is_error(self) -> bool:
        return self.level == self.ERROR

    def as_dict(self) -> dict:
        """For handing to the web planner, which speaks JSON."""
        return {"level": self.level,
                "message": self.message,
                "step": self.step,
                "suggestion": self.suggestion,
                "time_ms": self.time_ms}

    def __str__(self):
        where = "" if self.step is None else " in step {0}".format(self.step + 1)
        text = "{0}{1}: {2}".format(self.level, where, self.message)

        if self.suggestion is not None:
            text += "\n    try: {0}".format(self.suggestion)

        return text

    def __repr__(self):
        return "Diagnostic({0})".format(self)
