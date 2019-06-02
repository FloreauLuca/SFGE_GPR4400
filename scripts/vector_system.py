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

    pextendA = p2Vec2()
    pextendB = p2Vec2()
    ppositionA = p2Vec2()
    ppositionB = p2Vec2()

    extendA = Vec2f()
    extendB = Vec2f()
    positionA = Vec2f()
    positionB = Vec2f()

    cornerTR = Vec2f()
    cornerBR = Vec2f()
    cornerTL = Vec2f()
    cornerBL = Vec2f()

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

        self.positionA = Vec2f(1, 2)
        self.positionB = Vec2f(2, 6)
        self.extendA = Vec2f(1, 5)
        self.extendB = Vec2f(1, 1)

        self.ppositionA = Physics2dManager.pixel2meter(self.positionA)
        self.ppositionB = Physics2dManager.pixel2meter(self.positionB)
        self.pextendA = Physics2dManager.pixel2meter(self.extendA)
        self.pextendB = Physics2dManager.pixel2meter(self.extendB)

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
        self.v = Vec2f(0, 10)
        self.v = self.v.rotate(self.rotation_speed*self.t)
        self.u = Vec2f(0, 10)

        self.pu = Physics2dManager.pixel2meter(self.u)
        self.pv = Physics2dManager.pixel2meter(self.v)
        self.result = Physics2dManager.meter2pixel(self.pv * p2Vec2.dot(self.pu, self.pv)/p2Vec2.dot(self.pv, self.pv))
        print("dot: " + str(Physics2dManager.meter2pixel(p2Vec2.dot(self.pu, self.pv))))
        print("dot / dot: " +  str(Physics2dManager.meter2pixel(p2Vec2.dot(self.pu, self.pv)/p2Vec2.dot(self.pv, self.pv))))
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

        """
        # Rotation and angle test
        self.v = Vec2f(10, -10)
        self.u = Vec2f(10, 10)
        self.pv = Physics2dManager.pixel2meter(self.v)
        self.pu = Physics2dManager.pixel2meter(self.u)
        self.pv = self.pv.rotate(self.t)
        self.result = Physics2dManager.meter2pixel(self.pv)

        """
        """
        # Affichage Extend
        self.pu = Physics2dManager.pixel2meter((Vec2f(self.extendA.x, 0)).rotate(self.rotation_speed*self.t))
        self.pv = Physics2dManager.pixel2meter((self.positionB-self.positionA))
        self.result = Physics2dManager.meter2pixel(self.pu * p2Vec2.dot(self.pv, self.pu) / p2Vec2.dot(self.pu, self.pu))
        """

        # Affichage refléxion
        self.v = Vec2f(0, 10)
        self.u = Vec2f(0, -1)

        self.pu = Physics2dManager.pixel2meter(self.u)
        self.pv = Physics2dManager.pixel2meter(self.v)
        self.result = Physics2dManager.meter2pixel(self.pv - (self.pu.normalized() * p2Vec2.dot(self.pv, self.pu.normalized())) * 2)

    def on_draw(self):
        # rotating vector

        graphics2d_manager.draw_vector(self.v, Vec2f(550, 300), Color.Green)
        graphics2d_manager.draw_vector(self.u, Vec2f(750, 300), Color.Green)
        graphics2d_manager.draw_vector(self.result, Vec2f(750, 500), Color.Red)

        # graphics2d_manager.draw_vector(self.test,Vec2f(750, 400), Color.Cyan)
        """
        graphics2d_manager.draw_vector(self.second, Vec2f(750,400), Color.Green)
        graphics2d_manager.draw_vector(self.minute,Vec2f(750, 400), Color.Cyan)
        graphics2d_manager.draw_vector(self.hour,Vec2f(750, 400), Color.Red)
        """
        """
        resize = 50
        graphics2d_manager.draw_vector(Vec2f(self.extendA.x, 0)*resize,self.positionA*resize, Color.Blue)
        graphics2d_manager.draw_vector(Vec2f(0, self.extendA.y)*resize,self.positionA*resize, Color.Blue)
        graphics2d_manager.draw_vector(Vec2f(self.extendB.x, 0)*resize,self.positionB*resize, Color.Blue)
        graphics2d_manager.draw_vector(Vec2f(0, self.extendB.y)*resize,self.positionB*resize, Color.Blue)

        graphics2d_manager.draw_vector(Vec2f(0, 1) * resize, Vec2f(0, 0) * resize, Color.White)
        graphics2d_manager.draw_vector(Vec2f(1, 0) * resize, Vec2f(0, 0) * resize, Color.White)
    
        self.pu = Physics2dManager.pixel2meter((Vec2f(self.extendA.x, 0)).rotate(self.rotation_speed * self.t))
        self.pv = Physics2dManager.pixel2meter((self.positionB - self.positionA))
        self.result = Physics2dManager.meter2pixel(self.pu * p2Vec2.dot(self.pv, self.pu) / p2Vec2.dot(self.pu, self.pu))
        graphics2d_manager.draw_vector(Physics2dManager.meter2pixel(self.pu)*resize,self.positionA*resize, Color.Red)
        graphics2d_manager.draw_vector(Physics2dManager.meter2pixel(self.pv)*resize,self.positionA*resize, Color.Red)
    
        if (self.result.magnitude/Physics2dManager.meter2pixel(self.pu).magnitude) > 1 or (self.result.magnitude/Physics2dManager.meter2pixel(self.pu).magnitude) < -1:
            graphics2d_manager.draw_vector(self.result * resize, self.positionA * resize, Color.Red)
        else:
            graphics2d_manager.draw_vector(self.result * resize, self.positionA * resize, Color.Green)
    
        self.pu = Physics2dManager.pixel2meter((Vec2f(0, self.extendA.y)).rotate(self.rotation_speed * self.t))
        self.pv = Physics2dManager.pixel2meter((self.positionB - self.positionA))
        self.result = Physics2dManager.meter2pixel(self.pu * p2Vec2.dot(self.pv, self.pu) / p2Vec2.dot(self.pu, self.pu))
        graphics2d_manager.draw_vector(Physics2dManager.meter2pixel(self.pu) * resize, self.positionA * resize, Color.Red)
        if (self.result.magnitude/Physics2dManager.meter2pixel(self.pu).magnitude) > 1 or (self.result.magnitude/Physics2dManager.meter2pixel(self.pu).magnitude) < -1:
            graphics2d_manager.draw_vector(self.result * resize, self.positionA * resize, Color.Red)
        else:
            graphics2d_manager.draw_vector(self.result * resize, self.positionA * resize, Color.Green)
    
        self.pu = Physics2dManager.pixel2meter((Vec2f(self.extendB.x, 0)).rotate(self.rotation_speed * self.t))
        self.pv = Physics2dManager.pixel2meter((self.positionA - self.positionB))
        self.result = Physics2dManager.meter2pixel(self.pu * p2Vec2.dot(self.pv, self.pu) / p2Vec2.dot(self.pu, self.pu))
        graphics2d_manager.draw_vector(Physics2dManager.meter2pixel(self.pu) * resize, self.positionB * resize, Color.Red)
    
        if (self.result.magnitude/Physics2dManager.meter2pixel(self.pu).magnitude) > 1 or (self.result.magnitude/Physics2dManager.meter2pixel(self.pu).magnitude) < -1:
            graphics2d_manager.draw_vector(self.result * resize, self.positionB * resize, Color.Red)
        else:
            graphics2d_manager.draw_vector(self.result * resize, self.positionB * resize, Color.Green)
    
        self.pu = Physics2dManager.pixel2meter((Vec2f(0, self.extendB.y)).rotate(self.rotation_speed * self.t))
        self.pv = Physics2dManager.pixel2meter((self.positionA - self.positionB))
        self.result = Physics2dManager.meter2pixel(self.pu * p2Vec2.dot(self.pv, self.pu) / p2Vec2.dot(self.pu, self.pu))
        graphics2d_manager.draw_vector(Physics2dManager.meter2pixel(self.pu) * resize, self.positionB * resize, Color.Red)
    
        if (self.result.magnitude/Physics2dManager.meter2pixel(self.pu).magnitude) > 1 or (self.result.magnitude/Physics2dManager.meter2pixel(self.pu).magnitude) < -1:
            graphics2d_manager.draw_vector(self.result * resize, self.positionB * resize, Color.Red)
        else:
            graphics2d_manager.draw_vector(self.result * resize, self.positionB * resize, Color.Green)
        """
