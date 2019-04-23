from SFGE import *
from typing import List

shape_manager = graphics2d_manager.shape_manager


class ContactDebugSystem(System):

    entities: List[int]
    contact_count: List[int]

    def init(self):
        self.entities = entity_manager.get_entities_with_type(System.Shape)
        self.contact_count = [0 for i in range(len(self.entities))]
        for entity in self.entities:
            shape = shape_manager.get_component(entity)
            shape.set_fill_color(Color.Red)

    def fixed_update(self):
        for i in range(len(self.entities)):
            shape: Shape = shape_manager.get_component(self.entities[i])
            count = self.contact_count[i]
            if count > 0:
                shape.set_fill_color(Color.Green)
            else:
                shape.set_fill_color(Color.Magenta)

    def on_contact(self, c1, c2, enter):
        # print("Contact between {0} and {1} with enter: {2}".format(str(c1), str(c2), str(enter)))
        if enter:
            self.contact_count[self.entities.index(c1.entity)] += 1
            self.contact_count[self.entities.index(c2.entity)] += 1

        else:
            self.contact_count[self.entities.index(c1.entity)] -= 1
            self.contact_count[self.entities.index(c2.entity)] -= 1
