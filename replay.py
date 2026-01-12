import numpy as np
from scene.scene import Scene
from scene.record import Record
from resources import example_bodies, example_robots

np.set_printoptions(precision=2, suppress=True)

scene = Scene()
record = Record("data/test.pkl")

ground = record.objects["ground"]
scene.add_body(ground)

robot = record.objects["robot"]
scene.add_system(robot)

if "reference" in record.objects.keys():
    scene.add_reference()
    scene.reference.pose = record.frames[0]["reference"]


def update(dt):
    frame = record.get_next_frame(dt)

    for b, p in zip(robot.links(), frame["robot"]):
        b.pose = p

    scene.reference.pose = frame["reference"]

    scene.external_force.curr_body, scene.external_force.mouse, scene.external_force.anchor = frame["force"]
    scene.external_force.apply()
    
    print(frame["jacobian"])
    

scene.run(update)

