import dill
import copy
from scene.scene import Scene

class Record:

    def __init__(self, scene: Scene, filepath: str = None):
        self.scene = scene

        self.objects = {}
        self.getters = {}
        self.frames = {}

        self.t = 0

        self.track_scene()

        if filepath != None:
            with open(filepath, "rb") as f:
                self.objects, self.frames = dill.load(f)
                self.getters = {}
                self.load_scene()
    
    def track(self, key: str, obj, data_getter):
        self.objects[key] = copy.deepcopy(obj)
        self.getters[key] = data_getter

    def note(self, dt):
        self.frames[self.t] = {}
        for key, getter in self.getters.items():
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

    def track_scene(self):
        self.track("bodies", self.scene.bodies, lambda: [b.pose for b in self.scene.bodies])
        self.track("reference", None, lambda: self.scene.reference.pose)
        self.track("external force", None, lambda: (self.scene.external_force.curr_body, self.scene.external_force.mouse, self.scene.external_force.anchor))

    def load_scene(self):
        if "bodies" in self.objects.keys():
            for b in self.objects["bodies"]:
                self.scene.add_body(b)

        if "reference" in self.objects.keys():
            self.scene.add_reference()
        
        self.update_scene(self.frames[0])

    def update_scene(self, frame: dict):
        if "bodies" in frame.keys():
            for b, p in zip(self.scene.bodies, frame["bodies"]):
                b.pose = p
        
        if "reference" in frame.keys():
            self.scene.reference.pose = frame["reference"]

        if "external force" in frame.keys():
            self.scene.external_force.curr_body, self.scene.external_force.mouse, self.scene.external_force.anchor = frame["external force"]
            self.scene.external_force.apply()

