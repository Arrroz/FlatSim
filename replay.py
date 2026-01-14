import numpy as np
from scene.scene import Scene
from scene.record import Record

np.set_printoptions(precision=2, suppress=True)

scene = Scene()

record = Record(scene, "data/test.pkl")


def update(dt):
    frame = record.get_next_frame(dt)
    record.update_scene(frame)
    
    print(frame["jacobian"])
    

scene.run(update)

