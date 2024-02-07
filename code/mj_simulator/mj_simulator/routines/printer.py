from .process_builder import TickerProcess
from multiprocessing import Manager


class Printer(TickerProcess):
    def __init__(self, freq=25, name="printer", digits=4, niceness=10, output=None):

        TickerProcess.__init__(self, freq=freq, name=name, niceness=niceness)
        self.labels = []
        self.label_size = 0
        manager = Manager()
        if output is None:
            self.output = manager.dict()
        else:
            self.output = output
        self.digits = digits

    def set_labels(self, labels):
        self.labels = labels
        for label in self.labels:
            self.output[label] = 0
        self.label_size = len(self.labels)

    def on_interrupt(self):
        print("\n")

    def set_data(self, dict_to_print):
        self.output.update(dict_to_print)

    def target(self):
        for label in self.labels:
            print(label, round(self.output[label], self.digits), end="\n", flush=True)

        print((self.label_size) * "\033[A", end="\r", flush=True)
