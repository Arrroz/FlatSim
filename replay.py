import numpy as np
from scene.scene import Scene
from scene.record import Record
from misc.plot import Plot

np.set_printoptions(precision=2, suppress=True)

scene = Scene()

record = Record(scene, "data/test.pkl")

time_data = [f["time"] for f in record.frames]
jacobians_left = [f["jacobian_left"] for f in record.frames]
svd_values_left = np.array([np.linalg.svd(j, compute_uv=False) for j in jacobians_left])
jacobians_right = [f["jacobian_right"] for f in record.frames]
svd_values_right = np.array([np.linalg.svd(j, compute_uv=False) for j in jacobians_right])

svd_left_plot = Plot(
    [(time_data, value, fr"$V_{{{i}}}$") for i, value in enumerate(svd_values_left.T)]
)
svd_right_plot = Plot(
    [(time_data, value, fr"$V_{{{i}}}$") for i, value in enumerate(svd_values_right.T)]
)

def update(dt):
    frame = record.get_next_frame(dt)
    record.update_scene(frame)
    svd_left_plot.update(record.curr_frame_id)
    svd_right_plot.update(record.curr_frame_id)

scene.run(update)

