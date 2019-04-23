import random
import math
from SFGE import *

body2d_manager = physics2d_manager.body2d_manager
sprite_manager = graphics2d_manager.sprite_manager
texture_manager = graphics2d_manager.texture_manager


class PlanetSystem(System):
    screen_size: Vector2f
    entity_nmb = None  # type: int
    center_mass = 1000.0
    planet_mass = 1.0
    gravity_const = 1000.0

    def init(self):
        self.entity_nmb = 256
        self.screen_size = engine.config.screen_size
        entity_manager.resize(self.entity_nmb)

        for i in range(self.entity_nmb):
            new_entity = entity_manager.create_entity(i+1)

            transform = transform2d_manager.add_component(new_entity)  # type: Transform2d
            transform.position = Vec2f(random.randint(0, self.screen_size.x), random.randint(0, self.screen_size.y))

            body2d = body2d_manager.add_component(new_entity)  # type: Body2d
            body2d.velocity = self.calculate_init_speed(transform)

            sprite_manager.create_component(new_entity, "data/sprites/round.png")

    def calculate_init_speed(self, transform):
        delta_to_center = self.screen_size / 2.0 - transform.position

        # perpendicular direction

        vel_dir = Vector2f(-delta_to_center.y, delta_to_center.x)
        vel_dir = vel_dir / vel_dir.magnitude

        # a = v^2 / r <=> v = sqrt(F / m * r)
        speed = math.sqrt(self.calculate_new_force(transform).magnitude / self.planet_mass * Physics2dManager.pixel2meter(
            delta_to_center.magnitude))
        return p2Vec2(vel_dir.x * speed, vel_dir.y * speed)

    def calculate_new_force(self, transform):
        delta_to_center = self.screen_size / 2.0 - transform.position
        r = delta_to_center.magnitude
        force = self.gravity_const * self.center_mass * self.planet_mass / (r * r)
        return Physics2dManager.pixel2meter(delta_to_center / delta_to_center.magnitude * force)

    def fixed_update(self):
        for i in range(self.entity_nmb):
            transform = transform2d_manager.get_component(i+1)  # type: Transform2d

            body2d = body2d_manager.get_component(i+1)  # type: Body2d
            body2d.apply_force(self.calculate_new_force(transform))
