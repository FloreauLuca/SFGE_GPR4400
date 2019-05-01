from SFGE import *
from typing import List

body2d_manager = physics2d_manager.body2d_manager


class GizmoCollider(System):

    bodies_entites: list

    def init(self):
        self.bodies_entites = entity_manager.get_entities_with_type(System.Body)
        for entity in self.bodies_entites:
            body = body2d_manager.get_component(entity)
            shape = body.shape
            shape.set_fill_color(Color.Red)

    def fixed_update(self):
        for entity in self.bodies_entites:
            body_entities: Body2d = physics2d_manager.body2d_manager.get_component(entity)
            print("Ray fraction: " + str(body_entities.body.shape))
            body_entities.body.shape.set_fill_color(Color.Red)

            
    """
    def on_contact(self, c1, c2, enter):
        # print("Contact between {0} and {1} with enter: {2}".format(str(c1), str(c2), str(enter)))
        if enter:
            self.contact_count[self.entities.index(c1.entity)] += 1
            self.contact_count[self.entities.index(c2.entity)] += 1

        else:
            self.contact_count[self.entities.index(c1.entity)] -= 1
            self.contact_count[self.entities.index(c2.entity)] -= 1
    """
