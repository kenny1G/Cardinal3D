
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    //
    // The input screen_coord is a normalized screen coordinate [0,1]^2
    //
    // You need to transform this 2D point into a 3D position on the sensor plane, which is
    // located one unit away from the pinhole in camera space (aka view space).
    //
    // You'll need to compute this position based on the vertial field of view
    // (vert_fov) of the camera, and the aspect ratio of the output image (aspect_ratio).
    //
    // Tip: compute the ray direction in view space and use
    // the camera space to world space transform (iview) to transform the ray back into world space.
    Vec4 camera_origin = Vec4(0.0f, 0.0f, 0.0f, 1.0f);
    float vert_fov_rad = (vert_fov * PI_F) / 180.0f;
    float img_height = 2.0f * tan(vert_fov_rad / 2.0f);
    float img_width = img_height * aspect_ratio;

    float sensor_x = (screen_coord.x - 0.5f) * img_width;
    float sensor_y = (screen_coord.y - 0.5f) * img_height;
    Vec4 camera_dir = Vec4(sensor_x, sensor_y, -1.0f, 1.0f);

    Vec3 ray_origin = (iview * camera_origin).project();
    Vec3 ray_dir = (iview * camera_dir).project();

    return Ray(ray_origin, ray_dir - ray_origin);
}
