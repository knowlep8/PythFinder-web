from pythfinder.Trajectory.Control.Controllers.PIDCoefficients import *

# imported under a private name: this module is star-imported, and a plain
# 'import time' would push the name 'time' into every namespace that does so
import time as _time

# generic PID controller


# Was pygame.time.get_ticks(), which counts milliseconds since pygame.init().
# This keeps that meaning -- whole milliseconds since the program started -- so
# the controller behaves as before, without the library needing pygame.
_START = _time.monotonic()

def milliseconds_since_start() -> int:
    return int((_time.monotonic() - _START) * 1000)

class PIDController():
    def __init__(self, 
                 coefficients: PIDCoefficients):
        
        self.__coeff = coefficients

        self.__proportional = 0
        self.__derivative = 0
        self.__integral = 0

        self.__current_time = 0

        self.__past_error = 0
        self.__past_time = 0
    
    def set(self, 
            coefficients: PIDCoefficients):
        self.__coeff = coefficients

    def calculate(self, 
                  error: int | float):
        
        self.__current_time = milliseconds_since_start()
        elapsed = self.__current_time - self.__past_time

        self.__proportional = error

        # The clock counts whole milliseconds, so two calls can land in the same
        # one -- at the simulator's 1000 FPS ceiling that happens constantly.
        # Dividing by that gap would raise ZeroDivisionError, so the previous
        # derivative is held instead: no time has passed, so nothing has changed.
        if elapsed > 0:
            self.__derivative = (error - self.__past_error) / elapsed

        self.__integral += error

        power = self.__proportional * self.__coeff.kP + self.__integral * self.__coeff.kI + self.__derivative * self.__coeff.kD 

        self.__past_time = self.__current_time
        self.__past_error = error

        return power

