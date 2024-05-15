from helper_classes import *
import matplotlib.pyplot as plt

def render_scene(camera, ambient, lights, objects, screen_size, max_depth):
    width, height = screen_size
    ratio = float(width) / height
    screen = (-1, 1 / ratio, 1, -1 / ratio)  # left, top, right, bottom

    image = np.zeros((height, width, 3))

    for i, y in enumerate(np.linspace(screen[1], screen[3], height)):
        for j, x in enumerate(np.linspace(screen[0], screen[2], width)):
            # screen is on origin
            pixel = np.array([x, y, 0])
            origin = camera
            direction = normalize(pixel - origin)
            ray = Ray(origin, direction)
            
            intersection = get_intersection_tuple(ray, objects)
            if intersection is not None:
                hit_point, hit_object = intersection    
                color = get_color(hit_point, hit_object, ray, ambient, lights, objects, 0, max_depth)
            else:
                color = np.zeros(3)    
            # We clip the values between 0 and 1 so all pixel values will make sense.
            image[i, j] = np.clip(color,0,1)

    return image

def get_color(hit, hit_object, ray, ambient, lights, objects, depth, max_depth):
    color = np.zeros(3)
    color += calc_amibient(ambient, hit_object)
    
    for light in lights:
        if is_shadowed(hit, light, objects):
            continue
        color += calc_diffuse(light, hit, hit_object)
        color += calc_specular(light, hit, hit_object, ray)
        
    depth += 1
    if depth >= max_depth:
        return color
    
    r_ray = Ray(hit, reflected(ray.direction, hit_object.normal))
    r_ray_intersection = get_intersection_tuple(r_ray, objects)
    if r_ray_intersection is not None:
        r_hit, r_hit_object = r_ray_intersection
        color += hit_object.reflection * get_color(r_hit, r_hit_object, r_ray, ambient, lights, objects, depth, max_depth)
        
    return color  


def get_intersection_tuple(ray, objects):
    hit_object, distance = ray.nearest_intersected_object(objects)
    if hit_object is not None:
        hit_point = (ray.origin + distance * ray.direction) + (0.01 * hit_object.normal)
        return hit_point, hit_object
    return None

def is_shadowed(hit, light, objects):
    light_ray = light.get_light_ray(hit)
    intersection = light_ray.nearest_intersected_object(objects)
    if intersection is None:
        return False
    else:
        closest_object, distance = intersection
        if distance < light.get_distance_from_light(hit):
            return True
    
    return False    
    

def calc_amibient(ambient, object : Object3D):
    return object.ambient * ambient

def calc_diffuse(light, hit, object : Object3D):
    ray = light.get_light_ray(hit)
    angle_cosine = np.dot(normalize(object.normal), normalize(ray.direction))
    diffuse_intensity = object.diffuse * light.get_intensity(hit) * angle_cosine
    
    return diffuse_intensity

def calc_specular(light, hit, object : Object3D, view_ray : Ray):
    view_point = view_ray.origin
    light_ray = light.get_light_ray(hit)
    reflected_vector = reflected(normalize(-1 * light_ray.direction), object.normal)
    view = normalize(view_point - hit)
    angle_cosine_raised = np.dot(view, reflected_vector) ** object.shininess
    specular_intensity = object.specular * light.get_intensity(hit) * angle_cosine_raised
    
    return specular_intensity

def calc_reflection(hit, object : Object3D, ray, depth, max_depth):
    pass

def get_normal(hit, object):
    pass
        
# # Write your own objects and lights
# # TODO
# def your_own_scene():
#     camera = np.array([0,0,1])
#     lights = []
#     objects = []
#     return camera, lights, objects

def your_own_scene():
    # Define the camera
    camera = np.array([0, 0, 1])
    
    # Define light sources
    point_light = PointLight(intensity=np.array([1, 1, 1]), position=np.array([2, 2, 2]), kc=0.1, kl=0.1, kq=0.1)
    spot_light = SpotLight(intensity=np.array([1, 1, 1]), position=np.array([-2, 3, 1]), 
                           direction=np.array([2, -3, -1]), kc=0.1, kl=0.1, kq=0.1)
    
    # Define the pyramid with vertices
    pyramid_vertices = np.array([
        [-1, -1, -3],  # Base vertex A
        [1, -1, -3],   # Base vertex B
        [1, 1, -3],    # Base vertex C
        [-1, 1, -3],   # Base vertex D
        [0, 0, -2]     # Top vertex E (apex)
    ])
    pyramid = Pyramid(v_list=pyramid_vertices)
    pyramid.set_material(ambient=np.array([0.2, 0.8, 0.2]), diffuse=np.array([0.2, 1, 0.2]), 
                         specular=np.array([1, 1, 1]), shininess=10, reflection=0.7)
    
    # Define balls
    sphere1 = Sphere(center=np.array([2, 0, -4]), radius=0.5)
    sphere1.set_material(ambient=np.array([1, 0, 0]), diffuse=np.array([1, 0.4, 0.4]), 
                         specular=np.array([1, 1, 1]), shininess=50, reflection=0.65)
    
    sphere2 = Sphere(center=np.array([3, 0, -5]), radius=0.5)
    sphere2.set_material(ambient=np.array([0, 0, 1]), diffuse=np.array([0.4, 0.4, 1]), 
                         specular=np.array([1, 1, 1]), shininess=50, reflection=0.65)
    
    sphere3 = Sphere(center=np.array([2, 1, -4]), radius=0.5)
    sphere3.set_material(ambient=np.array([1, 1, 0]), diffuse=np.array([1, 1, 0.4]), 
                         specular=np.array([1, 1, 1]), shininess=50, reflection=0.65)
    
    # Define planes
    floor_plane = Plane(point=np.array([0, -1, 0]), normal=np.array([0, 1, 0]))
    floor_plane.set_material(ambient=np.array([0.1, 0.1, 0.1]), diffuse=np.array([0.6, 0.6, 0.6]), 
                             specular=np.array([1, 1, 1]), shininess=1000, reflection=0.5)
    
    background_plane = Plane(point=np.array([0, 0, -5]), normal=np.array([0, 0, 1]))
    background_plane.set_material(ambient=np.array([0.1, 0.1, 0.1]), diffuse=np.array([0.5, 0.5, 0.5]), 
                                  specular=np.array([1, 1, 1]), shininess=100, reflection=0.2)
    
    # List of objects and lights
    objects = [pyramid, sphere1, sphere2, sphere3, floor_plane, background_plane]
    lights = [point_light, spot_light]
    
    return camera, lights, objects
