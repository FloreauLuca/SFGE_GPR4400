from SFGE import *


class VectorSystem(System):
    t = 0.0
    rotation_speed = 0.0
    u = Vec2f()
    v = Vec2f()
    result = Vec2f()
    pu = p2Vec2()
    pv = p2Vec2()

    def init(self):
        self.t = 0.0
        self.rotation_speed = 45.0

    def update(self, dt):
        self.t += dt

        self.v = Vec2f(10, 10)
        self.v = self.v.rotate(self.rotation_speed*self.t)
        self.u = Vec2f(10, 10)

        self.pu = Physics2dManager.pixel2meter(self.u)
        self.pv = Physics2dManager.pixel2meter(self.v)
        self.result = Physics2dManager.meter2pixel(self.pu * p2Vec2.dot(self.pu, self.pv)/p2Vec2.dot(self.pv, self.pv))

    def on_draw(self):
        # rotating vector

        graphics2d_manager.draw_vector(self.v, Vec2f(400,400), Color.Green)
        graphics2d_manager.draw_vector(self.u, Vec2f(400,400), Color.Green)
        graphics2d_manager.draw_vector(self.result,
                                       Vec2f(400, 400), Color.Red)