from SFGE import *
from typing import List

shape_manager = graphics2d_manager.shape_manager
body2d_manager = physics2d_manager.body2d_manager


class Gravity(System):
    screen_size: Vector2f
    entity_nmb = None  # type: int
    center_mass = 1.0
    planet_mass = 1.0
    gravity_const = 9.18

    def init(self):
        self.entities = entity_manager.get_entities_with_type(System.Shape)
        for entity in self.entities:
            shape = shape_manager.get_component(entity)
            shape.set_fill_color(Color.Red)

    def calculate_new_force(self, transform):
        delta_to_center = self.screen_size - transform.position
        r = delta_to_center.magnitude
        force = self.gravity_const * self.center_mass * self.planet_mass / (r * r)
        return Physics2dManager.pixel2meter(delta_to_center / delta_to_center.magnitude * force)

    def fixed_update(self):
        for i in range(self.entity_nmb):
            transform = transform2d_manager.get_component(i+1)  # type: Transform2d

            body2d = body2d_manager.get_component(i+1)  # type: Body2d
            body2d.apply_force(self.calculate_new_force(transform))
