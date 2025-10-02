from multiprocessing import Manager


class SharedMemory:
    def __init__(self):
        manager = Manager()
        self.config = manager.dict()
        self.shared = manager.dict()

    def write(self, param: str, value=None):
        self.shared[param] = value

    def read(self, param: str):
        return self.shared.get(param, None)
