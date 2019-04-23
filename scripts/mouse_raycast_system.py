from typing import List

from SFGE import *
from SFGE import Vec2f


class MouseRaycastSystem(System):
    mouse_pos: Vec2f
    max_length: float
    lengths: List[float]
    angles: List[float]
    ray_nmb: int

    def init(self):
        self.ray_nmb = 20
        self.angles = [360 / self.ray_nmb * i for i in range(self.ray_nmb)]

        self.max_length = 1000.0
        self.lengths = [self.max_length for i in range(self.ray_nmb)]
        self.mouse_pos = Vec2f()

    def fixed_update(self):
        self.mouse_pos = input_manager.mouse.position
        for i in range(self.ray_nmb):
            # throw rays
            direction = Vec2f(0.0, 1.0)
            direction = direction.rotate(self.angles[i])
            self.lengths[i] = physics2d_manager.raycast(self.mouse_pos, direction, self.max_length)
            print("Ray fraction: "+str(self.lengths[i]))

    def on_draw(self):
        for i in range(self.ray_nmb):
            direction = Vec2f(0.0, 1.0)
            direction = direction.rotate(self.angles[i])
            graphics2d_manager.draw_line(self.mouse_pos, self.mouse_pos + direction * self.max_length * self.lengths[i],
                                         Color.White)
