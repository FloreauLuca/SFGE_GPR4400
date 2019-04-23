from SFGE import *


class VectorSystem(System):
    t = 0.0
    rotation_speed = 0.0
    v = Vec2f()

    def init(self):
        self.t = 0.0
        self.rotation_speed = 45.0

    def update(self, dt):
        self.t += dt

        self.v = Vec2f(10, 10)
        self.v = self.v.rotate(self.rotation_speed*self.t)

        self.u = Vec2f(10, 10)

    def on_draw(self):
        # rotating vector

        graphics2d_manager.draw_vector(self.v, Vec2f(400,400), Color.Green)
        graphics2d_manager.draw_vector(self.u, Vec2f(400,400), Color.Green)
        graphics2d_manager.draw_vector(self.u * Vec2f.dot(self.u, self.v)/Vec2f.dot(self.v,self.v),
                                       Vec2f(400, 400), Color.Red)