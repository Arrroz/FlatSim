import numpy as np
from scene.scene import Scene
from scene.record import Record
from misc.plot import Plot

np.set_printoptions(precision=2, suppress=True)

scene = Scene()

record = Record(scene, "data/test.pkl")

time_data = [f["time"] for f in record.frames]
torques = np.array([f["torques"] for f in record.frames])
jacobians = np.array([f["jacobian"] for f in record.frames])
forces = np.array([np.linalg.inv(j[:-1,:].T) @ t for j, t in zip(jacobians, torques)])
# svd_values_left = np.array([np.linalg.svd(j, compute_uv=False) for j in jacobians_left])
# jacobians_right = [f["jacobian_right"] for f in record.frames]
# svd_values_right = np.array([np.linalg.svd(j, compute_uv=False) for j in jacobians_right])
phases = np.array([f["phases"] for f in record.frames])
velocity = np.array([f["velocity"] for f in record.frames])
residuals = np.array([f["residual"] for f in record.frames])

# torques_plot = Plot(
#     [(time_data, value, fr"$\tau_{{{i}}}$") for i, value in enumerate(torques.T)]
# )
forces_plot = Plot(
    [(time_data, forces[:,0], fr"$f_x$"),
     (time_data, forces[:,1], fr"$f_y$"),
     (time_data, [np.linalg.norm(f) for f in forces], fr"$|f|$")]
)
forces_plot2 = Plot(
    [(forces[:,0], forces[:,1], fr"$f$")]
)
phases_plot = Plot([(time_data, p, label) for p, label in zip(phases.T, ["left", "middle", "right"])])
# velocity_plot = Plot(
#     [(time_data, velocity[:,0], fr"$v_x$"),
#      (time_data, velocity[:,1], fr"$v_y$"),
#      (time_data, [np.linalg.norm(v) for v in velocity], fr"$|v|$")]
# )
# residuals_plot = Plot(
#     [(time_data, value, fr"$r_{{{i}}}$") for i, value in enumerate(residuals.T)]
# )
# svd_left_plot = Plot(
#     [(time_data, value, fr"$V_{{{i}}}$") for i, value in enumerate(svd_values_left.T)]
# )
# svd_right_plot = Plot(
#     [(time_data, value, fr"$V_{{{i}}}$") for i, value in enumerate(svd_values_right.T)]
# )

def update(dt):
    frame = record.get_next_frame(dt)
    record.update_scene(frame)
    
    forces_plot.update(record.curr_frame_id)
    forces_plot2.update(record.curr_frame_id)
    phases_plot.update(record.curr_frame_id)
    # velocity_plot.update(record.curr_frame_id)
    # residuals_plot.update(record.curr_frame_id)
    # torques_plot.update(record.curr_frame_id)

scene.run(update)

