import pyglet
from physics import engine, body, utils
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

    def add_body(self, body: body.Body):
        if body in self.bodies:
            return
        
        self.bodies.append(body)
        self.camera.add_sprite(body.sprite_generator())
        self.engine.reset()
    
    def add_system(self, system: system.System):
        for l in system.links():
            self.add_body(l)
        self.constraints += [j for j in system.joints if not (j in self.constraints)]

        self.engine.reset()
        self.engine.collision_handler.add_collision_ignore_set(system.links())

    def apply_gravity(self, acc=utils.gravity):
        for b in self.bodies:
            if b.movable:
                b.apply_force(b.mass * acc)

    def apply_external_force(self):
        self._external_force.update()

    def add_reference(self):
        if self.reference == None:
            self.reference = reference.Reference(self._key_handler)
            self.camera.add_sprite(self.reference.sprite_generator())

    def show_collisions(self):
        self._collision_visuals.update(self.engine.collision_handler.collisions)

    def hide_collisions(self):
        self._collision_visuals.update([])

