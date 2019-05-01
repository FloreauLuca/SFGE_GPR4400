from SFGE import *


class StayOnscreenSystem(System):

    bodies_entites: list

    def init(self):
        self.bodies_entites = entity_manager.get_entities_with_type(System.Body)

    def fixed_update(self):
        config = engine.config
        screen_size = config.screen_size
        for entity in self.bodies_entites:
            transform = transform2d_manager.get_component(entity)
            body: Body2d = physics2d_manager.body2d_manager.get_component(entity)

            position = transform.position
            if (position.x < 0 and body.velocity.x < 0) or (position.x > screen_size.x and body.velocity.x > 0):
                body.velocity = p2Vec2(-body.velocity.x, body.velocity.y)
            if (position.y < 0 and body.velocity.y < 0) or (position.y > screen_size.y and body.velocity.y > 0):
                body.velocity = p2Vec2(body.velocity.x, -body.velocity.y)


