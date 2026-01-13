import dill
import copy
from scene.scene import Scene

class Record:

    def __init__(self, filepath: str = None):
        self.objects = {}
        self.getters = {}
        self.frames = {}

        self.t = 0

        if filepath != None:
            with open(filepath, "rb") as f:
                self.objects, self.frames = dill.load(f)

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

    def track_scene(self, scene: Scene):
        self.track("bodies", scene.bodies, lambda: [b.pose for b in scene.bodies])
        self.track("reference", None, lambda: scene.reference.pose)
        self.track("external force", None, lambda: (scene.external_force.curr_body, scene.external_force.mouse, scene.external_force.anchor))

    def load_scene(self, scene: Scene):
        if "bodies" in self.objects.keys():
            for b in self.objects["bodies"]:
                scene.add_body(b)

        if "reference" in self.objects.keys():
            scene.add_reference()
        
        self.update_scene(scene, self.frames[0])

    def update_scene(self, scene: Scene, frame: dict):
        if "bodies" in frame.keys():
            for b, p in zip(scene.bodies, frame["bodies"]):
                b.pose = p
        
        if "reference" in frame.keys():
            scene.reference.pose = frame["reference"]

        if "external force" in frame.keys():
            scene.external_force.curr_body, scene.external_force.mouse, scene.external_force.anchor = frame["external force"]
            scene.external_force.apply()

