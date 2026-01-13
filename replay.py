import numpy as np
from scene.scene import Scene
from scene.record import Record

np.set_printoptions(precision=2, suppress=True)

scene = Scene()
record = Record("data/test.pkl")

record.load_scene(scene)


def update(dt):
    frame = record.get_next_frame(dt)
    record.update_scene(scene, frame)
    
    print(frame["jacobian"])
    

scene.run(update)

