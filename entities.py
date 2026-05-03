import numpy as np
from geometry import Point, Rectangle, Circle, Ring
from typing import Union
import copy


class Entity:
    def __init__(self, center: Point, heading: float, movable: bool = True, friction: float = 0):
        self.center = center # this is x, y
        self.heading = heading
        self.movable = movable
        self.color = 'ghost white'
        self.collidable = True
        if movable:
            self.friction = friction
            self.velocity = Point(0,0) # this is xp, yp
            self.acceleration = 0 # this is vp (or speedp)
            self.angular_velocity = 0 # this is headingp
            self.inputSteering = 0
            self.inputAcceleration = 0
            self.max_speed = np.inf
            self.min_speed = 0
    
    @property
    def speed(self) -> float:
        return self.velocity.norm(p = 2) if self.movable else 0
    
    def set_control(self, inputSteering: float, inputAcceleration: float):
        self.inputSteering = inputSteering
        self.inputAcceleration = inputAcceleration
    
    @property
    def rear_dist(self) -> float: # distance between the rear wheels and the center of mass. This is needed to implement the kinematic bicycle model dynamics
        if isinstance(self, RectangleEntity):
            # only for this function, we assume
            # (i) the longer side of the rectangle is always the nominal direction of the car
            # (ii) the center of mass is the same as the geometric center of the RectangleEntity.
            return np.maximum(self.size.x, self.size.y) / 2.
        elif isinstance(self, CircleEntity):
            return self.radius
        elif isinstance(self, RingEntity):
            return (self.inner_radius + self.outer_radius) / 2.
        raise NotImplementedError
    
    def tick(self, dt: float):
        if self.movable:
            speed = self.speed
            heading = self.heading
        
            # Kinematic bicycle model dynamics based on
            # "Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design" by
            # Jason Kong, Mark Pfeiffer, Georg Schildbach, Francesco Borrelli
            lr = self.rear_dist
            lf = lr # we assume the center of mass is the same as the geometric center of the entity
            beta = np.arctan(lr / (lf + lr) * np.tan(self.inputSteering))
            
            new_angular_velocity = speed * self.inputSteering # this is not needed and used for this model, but let's keep it for consistency (and to avoid if-else statements)
            new_acceleration = self.inputAcceleration - self.friction
            new_speed = np.clip(speed + new_acceleration * dt, self.min_speed, self.max_speed)
            new_heading = heading + ((speed + new_speed)/lr)*np.sin(beta)*dt/2.
            angle = (heading + new_heading)/2. + beta
            new_center = self.center + (speed + new_speed)*Point(np.cos(angle), np.sin(angle))*dt / 2.
            new_velocity = Point(new_speed * np.cos(new_heading), new_speed * np.sin(new_heading))
            
            '''
            # Point-mass dynamics based on
            # "Active Preference-Based Learning of Reward Functions" by
            # Dorsa Sadigh, Anca D. Dragan, S. Shankar Sastry, Sanjit A. Seshia
            
            new_angular_velocity = speed * self.inputSteering
            new_acceleration = self.inputAcceleration - self.friction * speed
            
            new_heading = heading + (self.angular_velocity + new_angular_velocity) * dt / 2.
            new_speed = np.clip(speed + (self.acceleration + new_acceleration) * dt / 2., self.min_speed, self.max_speed)
            
            new_velocity = Point(((speed + new_speed) / 2.) * np.cos((new_heading + heading) / 2.),
                                    ((speed + new_speed) / 2.) * np.sin((new_heading + heading) / 2.))
            
            new_center = self.center + (self.velocity + new_velocity) * dt / 2.
            
            '''
            
            self.center = new_center
            self.heading = np.mod(new_heading, 2*np.pi) # wrap the heading angle between 0 and +2pi
            self.velocity = new_velocity
            self.acceleration = new_acceleration
            self.angular_velocity = new_angular_velocity
            
            self.buildGeometry()
    
    def buildGeometry(self): # builds the obj
        raise NotImplementedError
        
    def collidesWith(self, other: Union['Point','Entity']) -> bool:
        if isinstance(other, Entity):
            return self.obj.intersectsWith(other.obj)
        elif isinstance(other, Point):
            return self.obj.intersectsWith(other)
        raise NotImplementedError
        
    def distanceTo(self, other: Union['Point','Entity']) -> float:
        if isinstance(other, Entity):
            return self.obj.distanceTo(other.obj)
        elif isinstance(other, Point):
            return self.obj.distanceTo(other)
        raise NotImplementedError
        
    def copy(self):
        return copy.deepcopy(self)
        
    @property
    def x(self):
        return self.center.x

    @property
    def y(self):
        return self.center.y
        
    @property
    def xp(self):
        return self.velocity.x

    @property
    def yp(self):
        return self.velocity.y
    
class RectangleEntity(Entity):
    def __init__(self, center: Point, heading: float, size: Point, movable: bool = True, friction: float = 0):
        super(RectangleEntity, self).__init__(center, heading, movable, friction)
        self.size = size
        self.buildGeometry()
    
    @property
    def edge_centers(self):
        edge_centers = np.zeros((4,2), dtype=np.float32)
        x = self.center.x
        y = self.center.y
        w = self.size.x
        h = self.size.y
        edge_centers[0] = [x + w / 2. * np.cos(self.heading), y + w / 2. * np.sin(self.heading)]
        edge_centers[1] = [x - h / 2. * np.sin(self.heading), y + h / 2. * np.cos(self.heading)]
        edge_centers[2] = [x - w / 2. * np.cos(self.heading), y - w / 2. * np.sin(self.heading)]
        edge_centers[3] = [x + h / 2. * np.sin(self.heading), y - h / 2. * np.cos(self.heading)]
        return edge_centers
        
    @property
    def corners(self):
        ec = self.edge_centers
        c = np.array([self.center.x, self.center.y])
        corners = []
        corners.append(Point(*(ec[1] + ec[0] - c)))
        corners.append(Point(*(ec[2] + ec[1] - c)))
        corners.append(Point(*(ec[3] + ec[2] - c)))
        corners.append(Point(*(ec[0] + ec[3] - c)))
        return corners
        
    def buildGeometry(self):
        C = self.corners
        self.obj = Rectangle(*C[:-1])
        
class DiffDriveEntity(RectangleEntity):
    """2WD Diferansiyel Sürüş kinematik modeli.
    
    set_control(v_left, v_right) ile sol ve sağ tekerlek hızları verilir.
    tick(dt) içinde ICC (Instantaneous Center of Curvature) tabanlı
    pozisyon ve heading güncellemesi yapılır.
    """

    def __init__(self, center: Point, heading: float, size: Point,
                 movable: bool = True, friction: float = 0):
        super().__init__(center, heading, size, movable, friction)
        if movable:
            self.v_L = 0.0  # Sol tekerlek hızı (m/s)
            self.v_R = 0.0  # Sağ tekerlek hızı (m/s)

    @property
    def track_width(self) -> float:
        """Tekerlekler arası mesafe (size.y = robotun genişliği)."""
        return self.size.y

    def set_control(self, v_left: float, v_right: float):
        """Sol ve sağ tekerlek hızlarını ayarla.
        
        Args:
            v_left: Sol tekerlek hızı (m/s)
            v_right: Sağ tekerlek hızı (m/s)
        """
        self.v_L = v_left
        self.v_R = v_right

    def tick(self, dt: float):
        if not self.movable:
            return

        # Friction uygula (her tekerleğe ayrı ayrı sönümleme)
        v_L = self.v_L
        v_R = self.v_R
        if self.friction > 0:
            v_L = v_L - np.sign(v_L) * self.friction * dt
            v_R = v_R - np.sign(v_R) * self.friction * dt
            # Friction hızı sıfırın ötesine taşımasın
            if np.sign(v_L) != np.sign(self.v_L) and self.v_L != 0:
                v_L = 0.0
            if np.sign(v_R) != np.sign(self.v_R) and self.v_R != 0:
                v_R = 0.0

        # Tekerlek hızlarını sınırla
        v_L = np.clip(v_L, -self.max_speed, self.max_speed)
        v_R = np.clip(v_R, -self.max_speed, self.max_speed)

        L = self.track_width
        EPS = 1e-8

        # Doğrusal ve açısal hız
        v = (v_R + v_L) / 2.0
        omega = (v_R - v_L) / L if L > EPS else 0.0

        x = self.center.x
        y = self.center.y
        theta = self.heading

        if abs(v_R - v_L) < EPS:
            # Durum 1: Düz çizgi hareketi (v_L ≈ v_R)
            x_new = x + v * np.cos(theta) * dt
            y_new = y + v * np.sin(theta) * dt
            theta_new = theta
        elif abs(v_R + v_L) < EPS:
            # Durum 2: Yerinde dönüş (v_L ≈ -v_R) — pure rotation
            x_new = x
            y_new = y
            theta_new = theta + omega * dt
        else:
            # Durum 3: Genel ICC tabanlı ark hareketi
            R = (L / 2.0) * (v_L + v_R) / (v_R - v_L)
            icc_x = x - R * np.sin(theta)
            icc_y = y + R * np.cos(theta)
            dtheta = omega * dt
            cos_dt = np.cos(dtheta)
            sin_dt = np.sin(dtheta)
            x_new = cos_dt * (x - icc_x) - sin_dt * (y - icc_y) + icc_x
            y_new = sin_dt * (x - icc_x) + cos_dt * (y - icc_y) + icc_y
            theta_new = theta + dtheta

        # State güncelle
        self.center = Point(x_new, y_new)
        self.heading = np.mod(theta_new, 2 * np.pi)
        self.velocity = Point(v * np.cos(theta_new), v * np.sin(theta_new))
        self.acceleration = 0
        self.angular_velocity = omega

        self.buildGeometry()


class CircleEntity(Entity):
    def __init__(self, center: Point, heading: float, radius: float, movable: bool = True, friction: float = 0):
        super(CircleEntity, self).__init__(center, heading, movable, friction)
        self.radius = radius
        self.buildGeometry()
        
    def buildGeometry(self):
        self.obj = Circle(self.center, self.radius)
                    
class RingEntity(Entity):
    def __init__(self, center: Point, heading: float, inner_radius: float, outer_radius: float, movable: bool = True, friction: float = 0):
        super(RingEntity, self).__init__(center, heading, movable, friction)
        self.inner_radius = inner_radius
        self.outer_radius = outer_radius
        self.buildGeometry()
        
    def buildGeometry(self):
        self.obj = Ring(self.center, self.inner_radius, self.outer_radius)