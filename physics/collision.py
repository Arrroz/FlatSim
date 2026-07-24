import numpy as np
from physics import body

class CollidableFeature():

    def __init__(self, anchor_x=0, anchor_y=0):
        self.parent = None # type: Collidable
        self.anchor = np.array([anchor_x, anchor_y])

    def update_pos(self):
        cos_theta = np.cos(self.parent.theta)
        sin_theta = np.sin(self.parent.theta)
        r_mat = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
        self.relative_pos = r_mat @ self.anchor
        self.pos = self.parent.pos + self.relative_pos

    def colliding(self, feature, tolerance):
        return []

    # helper functions for collision detection # TODO: should these be here?
    def p2p_vec(self, p1: np.ndarray, p2: np.ndarray): # vector between 2 points
        u = p2 - p1
        dist = np.linalg.norm(u)
        if dist == 0:
            return (np.array([1, 0]), 0)
        return (u / dist, dist)

    def l2p_dist(self, p: np.ndarray, line_p: np.ndarray, normal: np.ndarray):
        return np.dot(p - line_p, normal)

    def l2p_vecs(self, p: np.ndarray, line_p: np.ndarray, tangent: np.ndarray, normal: np.ndarray): # projections of a point in a line's tangent and normal # TODO: remove normal if not necessary (could use cross product with tangent instead)
        u = p - line_p
        return (np.dot(u, tangent), np.dot(u, normal))


class Collidable(body.Body):

    def __init__(self, collidable_features: list[CollidableFeature], *args, **kwargs):
        super().__init__(*args, **kwargs)

        for f in collidable_features:
            f.parent = self
            f.update_pos()
        self.collidable_features = collidable_features

    def update_features(self):
        for f in self.collidable_features:
            f.update_pos()


class Collision():

    def __init__(self, collidable1: Collidable, collidable2: Collidable, relative_pos1: np.ndarray, relative_pos2: np.ndarray, normal: np.ndarray, dist: float):
        self.collidable1 = collidable1
        self.collidable2 = collidable2
        self.relative_pos1 = relative_pos1
        self.relative_pos2 = relative_pos2
        self.normal = normal
        self.dist = dist


class CollisionHandler():

    def __init__(self, bodies: list[body.Body], tolerance=1e-3):
        self.tolerance = tolerance # separation distance below which a contact is reported
        self.reset(bodies)

    def reset(self, bodies: list[body.Body]):
        self.collidables = [b for b in bodies if isinstance(b, Collidable)] # type: list[Collidable]

        self.collidable_feature_pairs = [] # type: list[tuple[CollidableFeature]]
        for i in range(len(self.collidables)-1):
            collidable1 = self.collidables[i]

            for j in range(i+1, len(self.collidables)):
                collidable2 = self.collidables[j]

                for f1 in collidable1.collidable_features:
                    for f2 in collidable2.collidable_features:
                        self.collidable_feature_pairs.append((f1, f2))

        self.collisions = [] # type: list[Collision]

    def update_collisions(self):
        for c in self.collidables:
            c.update_features()

        self.collisions = []
        for f_pair in self.collidable_feature_pairs:
            for collision in f_pair[0].colliding(f_pair[1], self.tolerance):
                if self.is_collision_duplicate(collision):
                    continue

                self.collisions.append(collision)

    def is_collision_duplicate(self, collision: Collision):
        for c in self.collisions:
            if (collision.collidable1 == c.collidable1 and collision.collidable2 == c.collidable2 and
                np.all(collision.relative_pos1 == c.relative_pos1) and np.all(collision.relative_pos2 == c.relative_pos2)):
                return True

        return False

    def add_collision_ignore_set(self, ignore_set: list[body.Body]):
        ignore_set = [ign for ign in ignore_set if isinstance(ign, Collidable)] # filter out bodies that aren't Collidables

        for i in range(len(ignore_set)-1):
            ign1 = ignore_set[i]

            for j in range(i+1, len(ignore_set)):
                ign2 = ignore_set[j]

                for f1 in ign1.collidable_features:
                    for f2 in ign2.collidable_features:
                        if (f1, f2) in self.collidable_feature_pairs:
                            self.collidable_feature_pairs.remove((f1, f2))
                        if (f2, f1) in self.collidable_feature_pairs:
                            self.collidable_feature_pairs.remove((f2, f1))


# Beware of this file from this point on! It contains messy and undocumented... MATH!!!

class Circle(CollidableFeature):

    def __init__(self, anchor_x=0, anchor_y=0, radius=0):
        super().__init__(anchor_x, anchor_y)
        self.radius = radius

    def colliding(self, other: CollidableFeature, tolerance):
        if other.__class__ == Circle or other.__class__ == Point:
            normal, dist = self.p2p_vec(self.pos, other.pos)
            dist -= self.radius + other.radius

            if dist <= tolerance:
                rp1 = self.relative_pos + self.radius * normal
                rp2 = other.relative_pos - other.radius * normal
                return [Collision(self.parent, other.parent, rp1, rp2, normal, dist)]

            return []

        elif other.__class__ == Line:
            projection, dist = self.l2p_vecs(self.pos, other.p1, other.tangent, other.normal)

            if dist < 0: # removes weird collisions with polygon corners
                return []

            if projection < 0:
                normal, dist = self.p2p_vec(self.pos, other.p1)
                dist -= self.radius

                if dist <= tolerance:
                    rp1 = self.relative_pos + self.radius * normal
                    rp2 = other.relative_p1
                    return [Collision(self.parent, other.parent, rp1, rp2, normal, dist)]

            elif projection > other.length:
                normal, dist = self.p2p_vec(self.pos, other.p2)
                dist -= self.radius

                if dist <= tolerance:
                    rp1 = self.relative_pos + self.radius * normal
                    rp2 = other.relative_p2
                    return [Collision(self.parent, other.parent, rp1, rp2, normal, dist)]

            else:
                dist -= self.radius
                if dist <= tolerance:
                    rp1 = self.relative_pos - self.radius * other.normal
                    rp2 = other.relative_p1 + projection * other.tangent
                    return [Collision(self.parent, other.parent, rp1, rp2, -other.normal, dist)]

            return []

        else:
            return []


class Line(CollidableFeature):

    def __init__(self, anchor_x1=0, anchor_y1=0, anchor_x2=0, anchor_y2=0):
        super().__init__()
        self.anchor1 = np.array([anchor_x1, anchor_y1])
        self.anchor2 = np.array([anchor_x2, anchor_y2])
        diff = self.anchor2 - self.anchor1
        self.length = np.linalg.norm(diff)
        if self.length == 0:
            raise ValueError('Both points of Line Collidable are coincident')

    def update_pos(self):
        cos_theta = np.cos(self.parent.theta)
        sin_theta = np.sin(self.parent.theta)
        r_mat = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])

        self.relative_p1 = r_mat @ self.anchor1
        self.relative_p2 = r_mat @ self.anchor2
        self.p1 = self.parent.pos + self.relative_p1
        self.p2 = self.parent.pos + self.relative_p2
        self.tangent = (self.p2 - self.p1) / self.length
        self.normal = np.array([-self.tangent[1], self.tangent[0]]) # always points to the left of vector p1->p2

    def colliding(self, other: CollidableFeature, tolerance):
        if other.__class__ == Circle or other.__class__ == Point:
            return other.colliding(self, tolerance)

        elif other.__class__ == Line:
            collisions = []
            penetrating_collisions = []

            # the two segments intersect if the infinite-line crossing point lies within both extents
            det = self.tangent[1] * other.tangent[0] - self.tangent[0] * other.tangent[1]
            if det == 0: # parallel: no crossing
                intersect = False
            else:
                inv_mat = np.array([[other.tangent[1], -other.tangent[0]], [self.tangent[1], -self.tangent[0]]]) / det
                sol = inv_mat @ (self.p1 - other.p1)
                intersect = 0 <= sol[0] <= self.length and 0 <= sol[1] <= other.length

            proj1, dist1 = self.l2p_vecs(self.p1, other.p1, other.tangent, other.normal)
            proj2, dist2 = self.l2p_vecs(self.p2, other.p1, other.tangent, other.normal)

            for vertex_relative_pos, projection, dist in [(self.relative_p1, proj1, dist1), (self.relative_p2, proj2, dist2)]:
                if 0 <= projection <= other.length:
                    rp1 = vertex_relative_pos
                    rp2 = other.relative_p1 + projection * other.tangent
                    col = Collision(self.parent, other.parent, rp1, rp2, -other.normal, dist)
                    if 0 < dist <= tolerance:
                        collisions.append(col)
                    elif dist <= 0 and intersect:
                        penetrating_collisions.append(col)

            proj1, dist1 = self.l2p_vecs(other.p1, self.p1, self.tangent, self.normal)
            proj2, dist2 = self.l2p_vecs(other.p2, self.p1, self.tangent, self.normal)

            for vertex_relative_pos, projection, dist in [(other.relative_p1, proj1, dist1), (other.relative_p2, proj2, dist2)]:
                if 0 <= projection <= self.length:
                    rp1 = self.relative_p1 + projection * self.tangent
                    rp2 = vertex_relative_pos
                    col = Collision(self.parent, other.parent, rp1, rp2, self.normal, dist)
                    if 0 < dist <= tolerance:
                        collisions.append(col)
                    elif dist <= 0 and intersect:
                        penetrating_collisions.append(col)

            if len(penetrating_collisions) > 0:
                penetrating_collisions.sort(key = lambda c: c.dist, reverse = True)
                collisions.append(penetrating_collisions[0])

            return collisions

        else:
            return []


class Point(Circle):

    def __init__(self, anchor_x=0, anchor_y=0):
        super().__init__(anchor_x, anchor_y, 0)

    def colliding(self, other: CollidableFeature, tolerance):
        if other.__class__ == Point: # points shouldn't be able to collide as there is no defined norm
            return []

        return super().colliding(other, tolerance)
