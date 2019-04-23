

class Color:
    Red = 0
    Green = 0
    Blue = 0
    Yellow = 0
    Black = 0
    White = 0
    Magenta = 0
    Cyan = 0
    Transparent = 0

class Vector2f:
    """SFML and default SFGE Vector type when not working on the physic"""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.magnitude = 0.0


class Vec2f:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.magnitude = 0.0

    @staticmethod
    def dot(v1, v2):
        pass

    @staticmethod
    def angle_between(v1, v2):
        pass

    @staticmethod
    def lerp(v1, v2, t):
        pass



class Vector2i:
    """SFML and default SFGE Vector type when not working on the physic"""
    def __init__(self):
        self.x = 0
        self.y = 0
        self.magnitude = 0.0


class p2Vec2:
    """Box2d Vector type used for physics related"""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.magnitude = 0.0

    @staticmethod
    def dot(v1, v2):
        pass

    @staticmethod
    def cross(v1, v2):
        pass

    @staticmethod
    def lerp(v1, v2, f):
        pass

    @staticmethod
    def angle_between(v1, v2):
        pass

    def rotate(self, angle):
        pass

    def normalized(self):
        pass

    def normalized_self(self):
        pass


class p2Vec3:
    """Box2d Vector type used for physics related"""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.magnitude = 0.0

    @staticmethod
    def dot(v1, v2):
        pass

    @staticmethod
    def cross(v1, v2):
        pass

    @staticmethod
    def lerp(v1, v2, f):
        pass

    @staticmethod
    def angle_between(v1, v2):
        pass

    def rotate(self, angle):
        pass

    def normalized(self):
        pass

    def normalized_self(self):
        pass


class Sprite:
    def set_texture(self, texture):
        pass

    def load_texture(self, texture_name: str):
        pass

class Shape:
    def set_fill_color(self, color: Color):
        pass

class Timer:
    """Timer used for update loop and """
    def __init__(self, time, period):
        self.time = 0
        self.period = 0
    
    def reset(self):
        """Set the value of time to period"""
        pass
    
    def update(self, dt):
        """Update the timer of the dt given as parameter"""
        pass
    
    def get_current(self):
        """Get the current ratio of time between 0.0 and 1.0"""
        pass
    
    def get_current_time(self):
        """Get the current time from the start """
        pass
    
    def is_over(self):
        """Return the status of the termination of the timer"""
        pass


class ColliderData:
    def __init__(self):
        self.body = Body2d()
        self.collider = Collider()
        self.entity = 0

class System:
    def init(self):
        pass
    
    def update(self, dt):
        pass

    def fixed_update(self):
        pass
    
    def destroy(self):
        pass

    def on_contact(self, c1:ColliderData, c2:ColliderData, enter:bool):
        pass

    def on_draw(self):
        pass

class ComponentManager():
    def add_component(self, entity):
        pass

    def destroy_component(self, entity):
        pass

    def get_component(self, entity):
        pass

class SpriteManager(System):
    def add_component(self, entity):
        return Sprite()
    def get_component(self, entity:int):
        pass
class ShapeManager(System):
    def get_component(self, entity:int):
        pass

class TextureManager(System):
    pass    


class Graphics2dManager(System):
    def __init__(self):
        self.texture_manager = TextureManager()
        self.sprite_manager = SpriteManager()
        self.shape_manager = ShapeManager()

    def draw_line(self, from_vec:Vec2f, to_vec:Vec2f, color:Color):
        pass

    def draw_vector(self, v:Vec2f, origin_pos:Vec2f, color:Color):
        pass

class SceneManager(System):
    def load_scene(self, scene_name):
        pass


class Transform2dManager(System, ComponentManager):
    pass


class PythonEngine(System):
    def load_pycomponent(self, entity, script_path):
        pass


class EntityManager(System):

    def create_entity(self, wanted_entity):
        pass

    def destroy_entity(self, entity):
        pass

    def has_components(self, entity, component):
        pass

    def get_entities_with_type(self, componentType):
        pass


class Body2dManager(System, ComponentManager):
    pass


class Physics2dManager(System):
    body2d_manager = None # type: Body2dManager


class Configuration:
    def __init__(self):
        self.screen_size = Vec2f()

class Engine:
    def __init__(self):
        self.config = Configuration()


class Component:
    Sprite = 0
    Shape = 0
    Body = 0
    Sound = 0
    Transform2d = 0


class Transform2d():
    """Mandatory Component attached to the GameObject containing all the geometric important data of the GameObject"""
    def __init__(self):
        self.position = Vector2f()
        self.scale = Vector2f()
        self.euler_angle = 0.0


class Sound:
    def play(self):
        pass

    def stop(self):
        pass


class Body2d:
    def __init__(self):
        self.velocity = p2Vec2()
        self.magnitude = 0.0

    def apply_force(self, force:p2Vec2):
        pass

class Collider:
    pass

class KeyboardManager:
    class Key:
        Space = 0
        Up = 0
        Down = 0
        Left = 0
        Right = 0
    def is_key_held(self, key):
        pass

    def is_key_down(self, key):
        pass

    def is_key_up(self, key):
        pass

class MouseManager:
    def __init__(self):
        self.position = Vec2f()


class InputManager:
    keyboard = KeyboardManager()
    mouse = MouseManager()

engine = Engine()
input_manager = InputManager()
scene_manager = SceneManager()
transform2d_manager = Transform2dManager()
entity_manager = EntityManager()
physics2d_manager = Physics2dManager()
body2d_manager = Body2dManager()
graphics2d_manager = Graphics2dManager()
python_engine = PythonEngine()
