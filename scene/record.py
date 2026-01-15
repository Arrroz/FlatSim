import dill
import copy
from pyglet.window import key
from scene.scene import Scene

class Record:

    def __init__(self, scene: Scene, filepath: str = None):
        self.scene = scene

        self.objects = {}
        self.getters = {}
        self.frames = []

        self.t = 0
        self.playback_speed = 0
        self.curr_frame_id = 0

        self.track("time", None, lambda: self.t)
        self.track_scene()

        if filepath != None:
            self.load(filepath)
    
    def on_key_press(self, symbol, modifiers):
        match symbol:
            case key.P:
                if self.playback_speed == 0:
                    self.playback_speed = 1
                else:
                    self.playback_speed = 0
            case key.UP:
                if self.playback_speed < 2:
                    self.playback_speed += 0.5
            case key.DOWN:
                if self.playback_speed > -2:
                    self.playback_speed -= 0.5
            case key.RIGHT:
                if self.playback_speed == 0:
                    self.curr_frame_id += 1
                    self.t = self.frames[self.curr_frame_id]["time"]
            case key.LEFT:
                if self.playback_speed == 0:
                    self.curr_frame_id -= 1
                    self.t = self.frames[self.curr_frame_id]["time"]
    
    def track(self, key: str, obj, data_getter):
        self.objects[key] = copy.deepcopy(obj)
        self.getters[key] = data_getter

    def note(self, dt):
        new_frame = {}
        for key, getter in self.getters.items():
            new_frame[key] = copy.deepcopy(getter())
        self.frames.append(new_frame)

        self.t += dt
    
    def save(self, filepath: str):
        with open(filepath, "wb") as f:
            dill.dump((self.objects, self.frames), f)
    
    def load(self, filepath: str):
        with open(filepath, "rb") as f:
            self.objects, self.frames = dill.load(f)
            self.getters = {}

            self.curr_frame_id = 0

            self.load_scene()
            
            self.scene.camera.remove_handlers(self.scene)
            self.scene.camera.push_handlers(self.on_key_press)

            self.scene.play = True

    def get_next_frame(self, dt):
        if len(self.frames) == 0:
            raise IndexError("The Record has no frames")

        if self.curr_frame_id >= len(self.frames)-1 and self.playback_speed > 0:
            self.playback_speed = 0
        elif self.curr_frame_id <= 0 and self.playback_speed < 0:
            self.playback_speed = 0
        
        if self.playback_speed == 0:
            return self.frames[self.curr_frame_id]
        
        self.t += self.playback_speed * dt

        for i, f in enumerate(self.frames):
            if f["time"] >= self.t:
                self.curr_frame_id = i
                break
        
        return self.frames[self.curr_frame_id]

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

