import time
from datetime import datetime
from functools import wraps
from common import common_constants as const

def time_recorder(clsName: str):
    def internal_time_recorder(func):
        if const.TIME_RECORD_ENABLE is False:
            return func

        @wraps(func)
        def measureTimeFunc(*args, **kwargs):
            start_time = time.perf_counter()
            result = func(*args, **kwargs)
            end_time = time.perf_counter()
            elapsed_time = end_time - start_time
            print('[TimeRecorder]', clsName+':'+func.__name__, '{:.3f}'.format(elapsed_time), '[s]')
            return result
        return measureTimeFunc
    return internal_time_recorder