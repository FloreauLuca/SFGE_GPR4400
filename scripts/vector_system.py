from SFGE import *


class VectorSystem(System):
    t = 0.0
    rotation_speed = 0.0
    u = Vec2f()
    v = Vec2f()
    result = Vec2f()
    pu = p2Vec2()
    pv = p2Vec2()

    """
    tsec = 0.0
    tmin = 0.0
    thour = 0.0
    second = Vec2f()
    minute = Vec2f()
    hour = Vec2f()

    ptest = p2Vec2()
    test = Vec2f()
    ftest = 0.0
    
    """

    def init(self):
        """
        self.thour = 9.0
        self.tmin = 30.0
        self.tsec = 0.0
        self.t = self.thour * 3600 + self.tmin * 60 + self.tsec
        """
        self.t = 0;
        self.rotation_speed = 100
        # self.ftest = 0.0

    def update(self, dt):
        self.t += dt
        """
        # Affichage heure
        self.second = Vec2f(0,-5)
        self.minute = Vec2f(0,-7.5)
        self.hour = Vec2f(0,-10)
        self.thour = self.t/(3600)

        self.tmin = self.t/60

        self.tsec = self.t


        self.second = self.second.rotate(self.rotation_speed*self.tsec)
        self.minute = self.minute.rotate(self.rotation_speed*self.tmin)
        self.hour = self.hour.rotate(self.rotation_speed*self.thour)
        """

        """
        # Affichage projeté
        self.v = Vec2f(10, 10)
        self.v = self.v.rotate(self.rotation_speed*self.t)
        self.u = Vec2f(10, 10)

        self.pu = Physics2dManager.pixel2meter(self.u)
        self.pv = Physics2dManager.pixel2meter(self.v)
        self.result = Physics2dManager.meter2pixel(self.pu * p2Vec2.dot(self.pu, self.pv)/p2Vec2.dot(self.pv, self.pv))
        """

        """
        #Affichage Lerp

        if self.t>1 :
            self.t = 0

        self.v = Vec2f(10, -10)
        self.u = Vec2f(10, 10)
        self.pv = Physics2dManager.pixel2meter(self.v)
        self.pu = Physics2dManager.pixel2meter(self.u)
        self.pv = p2Vec2.lerp(self.pu,self.pv,self.t)
        self.result = Physics2dManager.meter2pixel(self.pv)
        
        """

        # """
        # Rotation and angle test
        self.v = Vec2f(10, -10)
        self.u = Vec2f(10, 10)
        self.pv = Physics2dManager.pixel2meter(self.v)
        self.pu = Physics2dManager.pixel2meter(self.u)
        if self.t > p2Vec2.angle_between(self.pu, self.pv):
            self.t = 0

        self.pv = self.pv.rotate(self.t)
        self.result = Physics2dManager.meter2pixel(self.pv)

        # """

    def on_draw(self):
        # rotating vector

        graphics2d_manager.draw_vector(self.v, Vec2f(750, 300), Color.Green)
        graphics2d_manager.draw_vector(self.u, Vec2f(750, 300), Color.Green)
        graphics2d_manager.draw_vector(self.result, Vec2f(750, 300), Color.Red)

        # graphics2d_manager.draw_vector(self.test,Vec2f(750, 400), Color.Cyan)
        """
        graphics2d_manager.draw_vector(self.second, Vec2f(750,400), Color.Green)
        graphics2d_manager.draw_vector(self.minute,Vec2f(750, 400), Color.Cyan)
        graphics2d_manager.draw_vector(self.hour,Vec2f(750, 400), Color.Red)
        """
