from scene.scene import Scene
from resources import example_bodies

scene = Scene()

scene.add_body(example_bodies.ground())

circle1 = example_bodies.wheel()
circle1.x = 2
circle1.y = 1
scene.add_body(circle1)

circle2 = example_bodies.wheel(radius=0.5)
circle2.x = -2
circle2.y = 1
scene.add_body(circle2)

rect1 = example_bodies.rectangle()
rect1.y = 2
scene.add_body(rect1)

rect2 = example_bodies.rectangle()
rect2.y = 1
scene.add_body(rect2)


def update(dt):
    scene.external_force.apply()

    scene.engine.step(dt)

    scene.show_collisions()


scene.run(update)

