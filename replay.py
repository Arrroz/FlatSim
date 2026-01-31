import numpy as np
from scene.scene import Scene
from scene.record import Record
from misc.plot import Plot

np.set_printoptions(precision=2, suppress=True)

scene = Scene()

record = Record(scene, "data/test.pkl")

time_data = [f["time"] for f in record.frames]
jacobian_00 = [f["jacobian"][0,0] for f in record.frames]
jacobian_01 = [f["jacobian"][0,1] for f in record.frames]
jacobian_10 = [f["jacobian"][1,0] for f in record.frames]
jacobian_11 = [f["jacobian"][1,1] for f in record.frames]
plot = Plot([
    (time_data, jacobian_00, "$J_{00}$"),
    (time_data, jacobian_01, "$J_{01}$"),
    (time_data, jacobian_10, "$J_{10}$"),
    (time_data, jacobian_11, "$J_{11}$")
])


def update(dt):
    frame = record.get_next_frame(dt)
    record.update_scene(frame)
    plot.update(record.curr_frame_id)
    

scene.run(update)

