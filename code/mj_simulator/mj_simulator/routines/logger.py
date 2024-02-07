from .process_builder import TickerProcess
from multiprocessing import Manager
from numpy import savetxt, array
import os


class Logger(TickerProcess):
    def __init__(
        self,
        freq=100,
        name="logger",
        file_name="log.csv",
        dir_name="logs",
        niceness=10,
        input_dict=None,
    ):

        TickerProcess.__init__(self, freq=freq, name=name, niceness=niceness)
        self.file_path = os.path.join(dir_name, file_name)
        self.labels = []
        manager = Manager()
        if input_dict is None:
            self.input_dict = manager.dict()
        else:
            self.input_dict = input_dict
        self.output_dict = manager.dict()

    def get_storage(self):
        return self.output_dict

    def set_labels(self, labels):
        self.labels = labels
        for label in self.labels:
            self.input_dict[label] = 0
            self.output_dict[label] = []

    def on_init(self):
        self.storage = {}
        for label in self.labels:
            self.storage[label] = []

    def target(self):
        for label in self.labels:
            self.storage[label].append(self.input_dict[label])
            self.output_dict[label] = self.storage[label]

    def set_data(self, dict_to_log):
        self.input_dict.update(dict_to_log)

    def save_file(self, file_path=None, with_header=False):
        output = []
        if not file_path:
            file_path = self.file_path

        for label in self.labels:
            output.append(self.output_dict[label])
        output_array = array(output).T

        header = ""
        if with_header:
            header = ",".join(self.labels)

        savetxt(file_path, output_array, delimiter=",", header=header, fmt="%s")
