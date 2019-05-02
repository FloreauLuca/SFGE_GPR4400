from SFGE import *
from typing import List

body2d_manager = physics2d_manager.body2d_manager


class GizmoCollider(System):
    bodies_entites: list
    top = float
    right = float
    left = float
    bottom = float

    def init(self):
        self.bodies_entites = entity_manager.get_entities_with_type(System.Body)
        for entity in self.bodies_entites:
            body = body2d_manager.get_component(entity)

        self.right = 1.0
        self.top = 1.0
        self.left = -1.0
        self.bottom = -1.0

    def fixed_update(self):

        for entity in self.bodies_entites:
            body_entities: Body2d = physics2d_manager.body2d_manager.get_component(entity)
            transform = transform2d_manager.get_component(entity)
            print("Python : position : " + str(transform.position.x) + ", " + str(transform.position.y))
            if transform.position.x != 400:
                self.right = 1.0 + Physics2dManager.pixel2meter(transform.position.x)
                self.top = 1.0 + Physics2dManager.pixel2meter(transform.position.y)
                self.left = -1.0 + Physics2dManager.pixel2meter(transform.position.x)
                self.bottom = -1.0 + Physics2dManager.pixel2meter(transform.position.y)
            print("Python : top : " + str(self.top) + "bottom : " + str(self.bottom) + "right : " + str(self.right) + "left : " + str(self.left))

    def on_draw(self):
        # rotating vector
        graphics2d_manager.draw_line(Physics2dManager.meter2pixel(p2Vec2(self.left, self.top)), Physics2dManager.meter2pixel(p2Vec2(self.right, self.top)), Color.Red)
        graphics2d_manager.draw_line(Physics2dManager.meter2pixel(p2Vec2(self.right, self.top)), Physics2dManager.meter2pixel(p2Vec2(self.right, self.bottom)), Color.Red)
        graphics2d_manager.draw_line(Physics2dManager.meter2pixel(p2Vec2(self.left, self.bottom)), Physics2dManager.meter2pixel(p2Vec2(self.left, self.top)), Color.Red)
        graphics2d_manager.draw_line(Physics2dManager.meter2pixel(p2Vec2(self.right, self.bottom)), Physics2dManager.meter2pixel(p2Vec2(self.left, self.bottom)), Color.Red)
