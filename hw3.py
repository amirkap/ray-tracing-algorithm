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

# # Write your own objects and lights
# # TODO
# def your_own_scene():
#     camera = np.array([0,0,1])
#     lights = []
#     objects = []
#     return camera, lights, objects

def your_own_scene():
    """
    This function defines a scene with a sims character like object and sets the lights and camera.

    Returns:
        camera: The position of the camera.
        lights: A list of lights in the scene.
        objects: A list of objects in the scene.
    """
    v_list = np.array([
        [-0.0835, 0.626165, -0.266335],  
        [-0.005665, 0.661335, -0.124165], 
        [0.080665, 0.655165, -0.266335], 
        [-0.017335, 0.792165, -0.238],  
        [0.038335, 0.3615, -0.1985]  
    ])

    diamond = Pyramid(v_list)
    diamond.set_material([0.1, 0.7, 0.1], [0.1, 0.7, 0.1], [0.3, 0.3, 0.3], 10, 0.9)  

    sphere = Sphere(center=[0, 0, 0], radius=0.25)
    sphere.set_material([0.8, 0.7, 0.5], [0.8, 0.7, 0.5], [0.3, 0.3, 0.3], 10, 0.3)  

    left_eye = Sphere(center=[-0.1, 0.08, 0.2], radius=0.03)
    left_eye.set_material([0, 0, 0], [0, 0, 0], [1, 1, 1], 10, 0.3)  

    right_eye = Sphere(center=[0.1, 0.08, 0.2], radius=0.03)
    right_eye.set_material([0, 0, 0], [0, 0, 0], [1, 1, 1], 10, 0.3)  

    plane = Plane([0, 1, 0], [0, -0.3, 0])
    plane.set_material([1, 1, 0], [1, 1, 0], [1, 1, 1], 1000, 0.8) 

    background = Plane([0, 0, 1], [0, 0, -3])
    background.set_material([0.5, 0.8, 1], [0.5, 0.8, 1], [0.5, 0.8, 1], 1000, 0) 

    house_list = np.array([[-1.7, -0.3, -1],
                           [-0.7, -0.3, -1],
                           [-1.7, 0.9, -1],
                           [-0.7, 0.9, -1]])

    house_lower_tri = Triangle(house_list[0], house_list[1], house_list[2])
    house_upper_tri = Triangle(house_list[1], house_list[3], house_list[2])

    house_lower_tri.set_material([1, 1, 1], [1, 1, 1], [0.1, 0.1, 0.1], 100, 0.5)
    house_upper_tri.set_material([1, 1, 1], [1, 1, 1], [0.1, 0.1, 0.1], 100, 0.5)

    roof_list = np.array([[-0.7,0.9,-1],
                   [-1.2,1.5,-1],
                   [-1.7,0.9,-1]])
    roof = Triangle(*roof_list)
    roof.set_material([1, 0, 0], [1, 0, 0], [0.1, 0.1, 0.1], 100, 0.5)

    # left window
    window1_list = np.array([[-1.6,0.4,-0.999],
                             [-1.4,0.4,-0.999],
                             [-1.6,0.6,-0.999],
                             [-1.4,0.6,-0.999]])

    window1_upper_tri = Triangle(window1_list[0], window1_list[1], window1_list[2])
    window1_lower_tri = Triangle(window1_list[1], window1_list[3], window1_list[2])

    window1_lower_tri.set_material([0, 0, 0], [0, 0, 0], [0.1, 0.1, 0.1], 100, 0.5)
    window1_upper_tri.set_material([0, 0, 0], [0, 0, 0], [0.1, 0.1, 0.1], 100, 0.5)

    # right window
    window2_list = np.array([[-1.0,0.4,-0.999],
                             [-0.8,0.4,-0.999],
                             [-1.0,0.6,-0.999],
                             [-0.8,0.6,-0.999]])

    window2_upper_tri = Triangle(window2_list[0], window2_list[1], window2_list[2])
    window2_lower_tri = Triangle(window2_list[1], window2_list[3], window2_list[2])

    window2_lower_tri.set_material([0, 0, 0], [0, 0, 0], [0.1, 0.1, 0.1], 100, 0.5)
    window2_upper_tri.set_material([0, 0, 0], [0, 0, 0], [0.1, 0.1, 0.1], 100, 0.5)

    door_list = np.array([[-1.35,-0.3,-0.999],
                        [-1.05,-0.3,-0.999],
                        [-1.35,0.2,-0.999],
                        [-1.05,0.2,-0.999]])

    door_upper_tri = Triangle(door_list[0], door_list[1], door_list[2])
    door_lower_tri = Triangle(door_list[1], door_list[3], door_list[2])

    door_lower_tri.set_material([0.5, 0.25, 0], [0.5, 0.25, 0], [0.1, 0.1, 0.1], 100, 0.5)
    door_upper_tri.set_material([0.5, 0.25, 0], [0.5, 0.25, 0], [0.1, 0.1, 0.1], 100, 0.5)

    door_knob = Sphere(center=[-1.27, -0.1, -0.998], radius=0.04)
    door_knob.set_material([0.8, 0.8, 0], [0.8, 0.8, 0], [0.3, 0.3, 0.3], 50, 0.4)

    point_light = PointLight(intensity=np.array([1, 1, 1]), position=np.array([2, 2, 2]), kc=0.1, kl=0.1, kq=0.1)
    directional_light = DirectionalLight(intensity=np.array([0.5, 0.5, 0.5]), direction=np.array([-1, -1, -1]))

    camera = np.array([0, 0, 1])  

    objects = [diamond, sphere, plane, background, left_eye, right_eye, house_lower_tri, house_upper_tri, roof,
               window1_upper_tri, window1_lower_tri, window2_upper_tri, window2_lower_tri, door_upper_tri, door_lower_tri, door_knob]
    lights = [point_light, directional_light]

    return camera, lights, objects