import pyglet
from physics import engine, utils
from graphics import camera
from system import system
from misc import external_force, reference, collision_visuals

class Scene:

    def __init__(self):
        # simulation
        self.engine = engine.Engine([], [])
        self.systems = [] # type: list[system.System]

        # window
        self.camera = camera.Camera(width=800, height=600)
        self._key_handler = pyglet.window.key.KeyStateHandler()
        self.camera.push_handlers(self._key_handler)

        # miscelaneous
        self._external_force = external_force.ExternalForce(self.camera)
        self._collision_visuals = collision_visuals.CollisionVisuals(self.camera)
        self.reference = None

        # time controls
        self.play = False

        # update function
        self.update_callback = None

    @property
    def bodies(self): return self.engine.bodies
    @bodies.setter
    def bodies(self, value): self.engine.bodies = value

    @property
    def constraints(self): return self.engine.constraints
    @constraints.setter
    def constraints(self, value): self.engine.constraints = value

    def run(self, update_callback):
        self.update_callback = update_callback

        # simulation setup
        for s in self.systems:
            self.bodies += [l for l in s.links() if not (l in self.bodies)]
            self.constraints += [j for j in s.joints if not (j in self.constraints)]
        self.engine.reset()
        for s in self.systems:
            self.engine.collision_handler.add_collision_ignore_set(s.links())

        # rendering setup
        for b in self.bodies:
            self.camera.add_sprite(b.sprite_generator())
        if self.reference:
            self.camera.add_sprite(self.reference.sprite_generator())

        # run scene
        pyglet.clock.schedule_interval(self.update, 1/120)
        pyglet.app.run()

    def update(self, dt):
        if self._key_handler[pyglet.window.key.Q]:
            self.play = True
        if self._key_handler[pyglet.window.key.W]:
            self.play = False

        if not self.play:
            return
        
        self.update_callback(dt)

    def apply_gravity(self, acc=utils.gravity):
        for b in self.bodies:
            if b.movable:
                b.apply_force(b.mass * acc)

    def apply_external_force(self):
        self._external_force.update()

    def add_reference(self):
        if self.reference == None:
            self.reference = reference.Reference(self._key_handler)

    def show_collisions(self):
        self._collision_visuals.update(self.engine.collision_handler.collisions)

    def hide_collisions(self):
        self._collision_visuals.update([])

