import logging as log
import logging
import tracemalloc
from collections import defaultdict
from time import perf_counter
from typing import Any, Callable


def measure_performance(func: Callable[..., Any]) -> Callable[..., Any]:
    def wrapper(*args: Any, **kwargs: Any) -> Any:
        start_time = perf_counter()
        value = func(*args, **kwargs)
        end_time = perf_counter()
        run_time = end_time - start_time
        print(f"Execution of {func.__name__} took {run_time:.5f} seconds,")
        return value

    return wrapper


def default_value_10_000_000() -> float:
    return 10_000_000.


# lock = Lock()
list_of_durations = defaultdict(list)
maximum_duration: dict[Any, float] = defaultdict(float)
minimum_duration: dict[Any, float] = defaultdict(default_value_10_000_000)
last_report_at: dict[Any, float] = defaultdict(float)
# maximum_duration = 0  # in seconds
# minimum_duration = 10_000_000  # in seconds
# last_report_at = perf_counter()


def measure_average_performance(func: Callable[..., Any]) -> Callable[..., Any]:
    """Report once every 5 minutes what the averages function duration is and what the max and min durations are."""

    def wrapper(*args: Any, **kwargs: Any) -> Any:
        global maximum_duration, minimum_duration, last_report_at
        start_time = perf_counter()
        value = func(*args, **kwargs)
        end_time = perf_counter()
        try:
            run_time = end_time - start_time
            if run_time > maximum_duration[func]:
                maximum_duration[func] = run_time
            if run_time < minimum_duration[func]:
                minimum_duration[func] = run_time
                list_of_durations[func].append(run_time)
            list_of_durations[func].append(run_time)

            now = perf_counter()
            if now - last_report_at[func] > 1 * 60 * 5 and list_of_durations[func]:
                average_duration = sum(list_of_durations[func]) / len(list_of_durations[func])
                log.info("*" * 30)
                log.info(f"Performance stats for {func.__name__}:")
                log.info(f"Avarage duration: {average_duration * 1000} ms, Maximum duration: {maximum_duration[func] * 1000} ms, Minimal duration: {minimum_duration[func] * 1000} ms")
                log.info("*" * 30)
                list_of_durations[func].clear()
                maximum_duration[func], minimum_duration[func] = 0, 10_000_000
                last_report_at[func] = perf_counter()
        except Exception:
            pass
        return value

    return wrapper


last_call_at = perf_counter()
last_report_call_frequency_at = perf_counter()
call_frequency_memory = defaultdict(list)


def measure_call_frequency(func: Callable[..., Any]) -> Callable[..., Any]:
    """Measure how often a given function is called."""

    def wrapper(*args: Any, **kwargs: Any) -> Any:
        global call_frequency_memory, last_call_at, last_report_call_frequency_at
        start_time = perf_counter()
        # time from last call
        time_from_last_call = start_time - last_call_at
        last_call_at = start_time
        call_frequency_memory[func].append(time_from_last_call)
        if start_time - last_report_call_frequency_at > 1 * 60 * 1 and call_frequency_memory[func]:
            average_call_time = sum(call_frequency_memory[func]) / len(call_frequency_memory[func])
            maximal_call_time = max(call_frequency_memory[func])
            print("*" * 50)
            print(f"Calls per second stats for {func.__name__}:")
            print(f"Average calls per second (CPS): {1 / average_call_time}, Minimal CPS: {1 / maximal_call_time}")
            print("*" * 50)
            call_frequency_memory[func].clear()
            last_report_call_frequency_at = start_time

        value = func(*args, **kwargs)
        return value

    return wrapper


def with_sql_exception_handling(func: Callable[..., Any]) -> Callable[..., Any]:
    """Try sql operation command, catch exception and finally close the cursor and the connection."""
    def wrapper(*args, **kwargs):
        try:
            result = func(*args, **kwargs)
            return result
        except Exception as e:
            log.critical(f"Couldn't perform sql operation: {repr(e)}")
            return None

    return wrapper


def trace_memory(func: Callable[..., Any]) -> Callable[..., Any]:
    """Trace RAM usage."""
    def wrapper(*args: Any, **kwargs: Any) -> Any:
        tracemalloc.start()
        value = func(*args, **kwargs)
        snapshot = tracemalloc.take_snapshot()
        top_stats = snapshot.statistics('lineno')
        log.info("*" * 50)
        log.info(f"TOP 10 RAM usage for {func.__name__}:")
        for stat in top_stats[:10]:
            log.info(stat)
        log.info("*" * 50)
        return value

    return wrapper
