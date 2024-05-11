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

            color = np.zeros(3)

            # This is the main loop where each pixel color is computed.
            # TODO

            
            # We clip the values between 0 and 1 so all pixel values will make sense.
            image[i, j] = np.clip(color,0,1)

    return image

def get_color(hit, ray, ambient, lights, objects, depth, max_depth):
    pass

def calc_amibient(ambient, hit, object : Object3D):
    return ambient * object.ambient

def calc_diffuse(light, hit, object : Object3D):
    ray = light.get_light_ray(hit)
    angle_cosine = np.dot(normalize(object.normal), normalize(ray.direction))
    diffuse_intensity = object.diffuse * light.get_intensity(hit) * max(angle_cosine, 0)
    
    return diffuse_intensity

def calc_specular(light, hit, object : Object3D, view_ray : Ray):
    view_point = view_ray.origin
    light_ray = light.get_light_ray(hit)
    reflected_vector = reflected(-1 * light_ray.direction, object.normal)
    view = normalize(view_point - hit)
    angle_cosine_raised = np.dot(view, reflected_vector) ** object.shininess
    specular_intensity = object.specular * light.get_intensity(hit) * max(angle_cosine_raised, 0)
    
    return specular_intensity

def calc_reflection(hit, object : Object3D, ray, depth, max_depth):
    pass

def get_normal(hit, object):
    pass
        
# Write your own objects and lights
# TODO
def your_own_scene():
    camera = np.array([0,0,1])
    lights = []
    objects = []
    return camera, lights, objects
