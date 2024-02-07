from multiprocessing import Process, Manager
from time import perf_counter
from os import nice


class TickerProcess:
    """
    Represents a ticker process that executes a target function at a specified frequency.

    Args:
        args (tuple, optional): Arguments to be passed to the target function (default: None).
        freq (int, optional): Frequency at which the target function is executed in Hz (default: 100).
        name (str, optional): Name of the ticker process (default: 'ticker').
        niceness (int, optional): Niceness value for process priority (default: 5).

    Attributes:
        name (str): Name of the ticker process.
        freq (int): Frequency at which the target function is executed in Hz.
        time (float): Elapsed time since the ticker process started.
        niceness (int): Niceness value for process priority.

    Methods:
        start(wait=True): Starts the ticker process.
        pause(): Pauses the ticker process.
        resume(): Resumes the ticker process.
        stop(): Terminates the ticker process.
        close(): Joins and terminates the ticker process.

    Note:
        The target function to be executed by the ticker process should be defined separately and
        assigned to the `target` method of the `TickerProcess` instance before calling `start()`.
    """

    def __init__(
        self,
        args=[],
        freq=100,
        name="ticker",
        niceness=5
    ):

        self._args = args

        self._process = None
        self._shared = Manager().Namespace()
        self._shared.is_active = False
        self.name = name
        self.freq = freq
        self.time = 0
        self.niceness = niceness

        self.build()

    def __del__(self) -> None:
        print(f"Process {self.name} was deleted from memory")

    def build(self):
        self._process = Process(target=self.ticker_process)

    def on_init(self):
        pass

    def on_while(self):
        pass

    def on_interrupt(self):
        pass

    def _set_priority(self):
        old_nice_value = nice(0)
        increment = self.niceness - old_nice_value
        self.niceness = nice(increment)

    def start(self, wait=True):
        """
        Starts the ticker process.

        Args:
            wait (bool, optional): Flag indicating whether to wait until the ticker process is active (default: True).
        """
        self._process.start()
        print(f"Process {self.name} is starting!")
        if wait:
            while not self._shared.is_active:
                pass

    def pause(self):
        """
        Pauses the ticker process.
        """
        self._shared.is_active = False

    def resume(self):
        """
        Resumes the ticker process.
        """
        self._shared.is_active = True

    def stop(self):
        """
        Terminates the ticker process.
        """
        self._process.terminate()
        print(f"Process {self.name} is terminated...")

    def close(self):
        """
        Joins and terminates the ticker process.
        """
        self._process.join(timeout=0.0)
        self._process.terminate()

    def ticker_process(self):
        """
        The main ticker process that executes the target function at the specified frequency.
        """
        self._set_priority()
        self.on_init()
        self._shared.is_active = True
        init_time = perf_counter()
        ticker_time = 0
        try:
            while True:
                if self._shared.is_active:
                    self.time = perf_counter() - init_time
                    self.on_while()
                    tick = self.time - ticker_time
                    if (tick) >= 1 / self.freq:
                        self.target(*self._args)
                        ticker_time = self.time

        except KeyboardInterrupt:
            self.on_interrupt()
            print(f"\n{self.name} process is over due KeyboardInterrupt...")

        except Exception as e:
            raise Exception(e)
