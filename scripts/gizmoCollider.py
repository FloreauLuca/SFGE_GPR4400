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

        self.right = 0.0
        self.top = 0.0
        self.left = 0.0
        self.bottom = 0.0


def fixed_update(self):

    for entity in self.bodies_entites:
        body_entities: Body2d = physics2d_manager.body2d_manager.get_component(entity)
        self.right = body_entities.aabb_topright.x
        self.top = body_entities.aabb_topright.y
        self.left = body_entities.aabb_bottomleft.x
        self.bottom = body_entities.aabb_bottomleft.y
    print("Python : right : " + self.right + "; top : " + self.top)


def on_draw(self):
    # rotating vector
    graphics2d_manager.draw_line(Physics2dManager.meter2pixel(p2Vec2(self.left, self.top)),
                                 Physics2dManager.meter2pixel(p2Vec2(self.right, self.top)), Color.red)
    graphics2d_manager.draw_line(Physics2dManager.meter2pixel(p2Vec2(self.right, self.top)),
                                 Physics2dManager.meter2pixel(p2Vec2(self.right, self.bottom)), Color.red)
    graphics2d_manager.draw_line(Physics2dManager.meter2pixel(p2Vec2(self.right, self.top)),
                                 Physics2dManager.meter2pixel(p2Vec2(self.left, self.top)), Color.red)
    graphics2d_manager.draw_line(Physics2dManager.meter2pixel(p2Vec2(self.right, self.bottom)),
                                 Physics2dManager.meter2pixel(p2Vec2(self.left, self.bottom)), Color.red)
