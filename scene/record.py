import dill
import copy

class Record:

    def __init__(self, filepath: str = None):
        self.tracked = {}
        self.frames = {}
        self.objects = {}
        self.t = 0

        if filepath != None:
            with open(filepath, "rb") as f:
                self.objects, self.frames = dill.load(f)

    def track(self, key: str, obj, data_getter):
        self.objects[key] = copy.deepcopy(obj)
        self.tracked[key] = data_getter

    def note(self, dt):
        self.frames[self.t] = {}
        for key, getter in self.tracked.items():
            self.frames[self.t][key] = copy.deepcopy(getter())

        self.t += dt
    
    def save(self, filepath: str):
        with open(filepath, "wb") as f:
            dill.dump((self.objects, self.frames), f)

    def get_next_frame(self, dt):
        if len(self.frames) == 0:
            raise IndexError("The Record has no frames")
        
        for t in self.frames.keys():
            if t >= self.t:
                break

        self.t += dt
        
        return self.frames[t]

