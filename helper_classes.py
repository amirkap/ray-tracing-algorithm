import numpy as np


# This function gets a vector and returns its normalized form.
def normalize(vector):
    return vector / np.linalg.norm(vector)


# This function gets a vector and the normal of the surface it hit
# This function returns the vector that reflects from the surface
def reflected(vector, axis):
    # reflected_vector = u - 2 * np.dot(u, axis) * axis
    reflected_vector = vector - 2 * np.dot(vector, axis) * axis
    return reflected_vector

## Lights


class LightSource:
    def __init__(self, intensity):
        self.intensity = intensity


class DirectionalLight(LightSource):

    def __init__(self, intensity, direction):
        super().__init__(intensity)
        self.direction = normalize(direction)

    # This function returns the ray that goes from a point to the light source
    def get_light_ray(self,intersection_point):
        return Ray(intersection_point, -1 * self.direction)

    # This function returns the distance from a point to the light source
    def get_distance_from_light(self, intersection):
        return np.inf

    # This function returns the light intensity at a point
    def get_intensity(self, intersection):
        return self.intensity


class PointLight(LightSource):
    def __init__(self, intensity, position, kc, kl, kq):
        super().__init__(intensity)
        self.position = np.array(position)
        self.kc = kc
        self.kl = kl
        self.kq = kq

    # This function returns the ray that goes from a point to the light source
    def get_light_ray(self,intersection):
        return Ray(intersection, normalize(self.position - intersection))

    # This function returns the distance from a point to the light source
    def get_distance_from_light(self,intersection):
        return np.linalg.norm(intersection - self.position)

    # This function returns the light intensity at a point
    def get_intensity(self, intersection):
        d = self.get_distance_from_light(intersection)
        return self.intensity / (self.kc + self.kl*d + self.kq * (d**2))


class SpotLight(LightSource):
    def __init__(self, intensity, position, direction, kc, kl, kq):
        super().__init__(intensity)
        self.direction = normalize(direction)
        self.position = np.array(position)
        self.kc = kc
        self.kl = kl
        self.kq = kq
        
    # This function returns the ray that goes from a point to the light source
    def get_light_ray(self, intersection):
        return Ray(intersection, self.position - intersection)

    def get_distance_from_light(self, intersection):
        return np.linalg.norm(intersection - self.position)

    def get_intensity(self, intersection):
        distance = self.get_distance_from_light(intersection)
        nominator = self.intensity * np.dot(normalize(intersection - self.position), normalize(self.direction))
        denominator = self.kc + self.kl * distance + self.kq * (distance ** 2)
        intensity = nominator / denominator
        
        return intensity

class Ray:
    def __init__(self, origin, direction):
        self.origin = origin
        self.direction = normalize(direction)

    # The function is getting the collection of objects in the scene and looks for the one with minimum distance.
    # The function should return the nearest object and its distance (in two different arguments)
    def nearest_intersected_object(self, objects):
        nearest_object = None
        min_t = np.inf
        for obj in objects:
            intersection = obj.intersect(self)
            if intersection is not None:
                t, _ = intersection
                if t < min_t and t > 0:
                    min_t = t
                    nearest_object = obj
        distance_vector = (self.origin + min_t * self.direction) - self.origin
                 
        return nearest_object, np.linalg.norm(distance_vector)


class Object3D:
    def set_material(self, ambient, diffuse, specular, shininess, reflection):
        self.ambient = ambient
        self.diffuse = diffuse
        self.specular = specular
        self.shininess = shininess
        self.reflection = reflection


class Plane(Object3D):
    def __init__(self, normal, point):
        self.normal = np.array(normal)
        self.point = np.array(point)

    def intersect(self, ray: Ray):
        v = self.point - ray.origin
        t = np.dot(v, self.normal) / (np.dot(self.normal, ray.direction) + 1e-6)
        if t > 0:
            return t, self
        else:
            return None


class Triangle(Object3D):
    """
        C
        /\
       /  \
    A /____\ B

    The fornt face of the triangle is A -> B -> C.
    
    """
    def __init__(self, a, b, c):
        self.a = np.array(a)
        self.b = np.array(b)
        self.c = np.array(c)
        self.normal = self.compute_normal()

    # computes normal to the trainagle surface. Pay attention to its direction!
    def compute_normal(self):
        normal = np.cross(self.b - self.a, self.c - self.a)
        return normalize(normal)

    def intersect(self, ray: Ray):
        plane = Plane(self.normal, self.a)
        res = plane.intersect(ray)
        if res is None:
            return None
        t, _ = res
        intersection_point = ray.origin + t * ray.direction
        if self.is_inside_triangle(intersection_point):
            return t, self
        else:
            return None
        
    def is_inside_triangle(self, point, epsilon=1e-6):
        # We use baricentric coordinates to check if the point is inside the triangle.
        ab = self.b - self.a
        ac = self.c - self.a
        pa = point - self.a
        pb = point - self.b
        pc = point - self.c
        area_abc = np.linalg.norm(np.cross(ab, ac))
        
        alpha = np.linalg.norm(np.cross(pb, pc)) / area_abc
        beta = np.linalg.norm(np.cross(pc, pa)) / area_abc
        gamma = np.linalg.norm(np.cross(pa, pb)) / area_abc
        
        is_triangle = (0 <= alpha <= 1) and (0 <= beta <= 1) and (0 <= gamma <= 1) and (np.abs(alpha + beta + gamma - 1) < epsilon)
        
        return is_triangle
   

class Pyramid(Object3D):
    """     
            D
            /\*\
           /==\**\
         /======\***\
       /==========\***\
     /==============\****\
   /==================\*****\
A /&&&&&&&&&&&&&&&&&&&&\ B &&&/ C
   \==================/****/
     \==============/****/
       \==========/****/
         \======/***/
           \==/**/
            \/*/
             E 
    
    Similar to Traingle, every from face of the diamond's faces are:
        A -> B -> D
        B -> C -> D
        A -> C -> B
        E -> B -> A
        E -> C -> B
        C -> E -> A
    """
    def __init__(self, v_list):
        self.v_list = v_list
        self.triangle_list  = self.create_triangle_list()

    def create_triangle_list(self) -> list[Triangle]:
        l = []
        t_idx = [
                [0,1,3],
                [1,2,3],
                [0,3,2],
                 [4,1,0],
                 [4,2,1],
                 [2,4,0]]
        for idx in t_idx:
            l.append(Triangle(self.v_list[idx[0]], self.v_list[idx[1]], self.v_list[idx[2]]))
             
        return l    

    def apply_materials_to_triangles(self):
        for t in self.triangle_list:
            t.set_material(self.ambient, self.diffuse, self.specular, self.shininess, self.reflection)
        pass

    def intersect(self, ray: Ray):
        min_t = np.inf
        min_obj = None
        for triangle in self.triangle_list:
            res = triangle.intersect(ray)
            if res is not None:
                t, obj = res
                if t < min_t and t > 0:
                    min_t = t
                    min_obj = obj
                    self.normal = obj.normal 
                    
        return min_t, min_obj if min_obj is not None else None
    
class Sphere(Object3D):
    def __init__(self, center, radius: float):
        self.center = center
        self.radius = radius

    def intersect(self, ray: Ray):
        d = ray.direction
        o = ray.origin
        c = self.center
        r = self.radius
        
        a = 1 # because d is normalized
        co = o - c
        b = 2 * np.dot(d, co)
        c = np.linalg.norm(co) ** 2 - r ** 2
        discriminant = b ** 2 - 4*a*c
        if discriminant < 0:
            return None
        t1 = (-b + np.sqrt(discriminant)) / (2*a)
        t2 = (-b - np.sqrt(discriminant)) / (2*a)
        
        if t1 > 0 and t2 > 0:
            mint_t = min(t1, t2)
        elif t1 > 0:
            mint_t = t1
        elif t2 > 0:
            mint_t = t2
        else:
            return None
        
        intersection_point = (o + mint_t * d)  
        self.set_normal(intersection_point)
        
        return mint_t, self
        
    def set_normal(self, point):
        self.normal = normalize(point - self.center)

