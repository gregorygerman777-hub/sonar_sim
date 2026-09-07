# distutils: language = c++
# cython: language_level=3
"""Cython surface for the C++ forward-scan sonar core.

Everything expensive lives in C++: the sub-ray casting, the beam-pattern
weighting and the intersection tests, which together are
O(azimuth_bins * elevation_subrays). Every physical parameter is exposed here,
so the physics can be retuned from Python without recompiling.

The render call drops the GIL. The C++ side threads over bearing, so a Python
process is free to do something else while a frame is forming.
"""

import numpy as np

from libcpp.vector cimport vector


cdef extern from "geometry.h":
    cdef cppclass Vec3:
        Vec3()
        double x
        double y
        double z
    cdef cppclass Texture:
        Texture()
        double amplitude
        double scale_m
        unsigned int seed
    cdef cppclass Hit:
        Hit()
        bint valid
        double t
        Vec3 normal
        double reflectivity
    cdef cppclass Plane:
        Plane()
        Vec3 point
        Vec3 normal
        double reflectivity
        Texture texture
    cdef cppclass Sphere:
        Sphere()
        Vec3 centre
        double radius
        double reflectivity
        Texture texture
    cdef cppclass Cylinder:
        Cylinder()
        Vec3 centre
        Vec3 axis
        double radius
        double half_length
        double reflectivity
        Texture texture
    cdef cppclass Scene:
        Scene()
        vector[Plane] planes
        vector[Sphere] spheres
        vector[Cylinder] cylinders
    Hit intersect_plane(const Plane&, const Vec3&, const Vec3&)
    Hit intersect_sphere(const Sphere&, const Vec3&, const Vec3&)
    Hit intersect_cylinder(const Cylinder&, const Vec3&, const Vec3&)
    double texture_factor(const Vec3&, const Texture&)


cdef extern from "physics.h":
    double beam_pattern(double, int, double, double)
    double thorp_absorption_db_per_km(double)
    double transmission_loss_db(double, double)


cdef extern from "simulator.h":
    cdef cppclass SonarConfig:
        SonarConfig()
        double frequency_hz
        int num_azimuth_bins
        int num_range_bins
        double horizontal_fov_deg
        double vertical_beamwidth_deg
        double max_range_m
        double speed_of_sound_mps
        int num_elevation_subrays
        int array_element_count
        double array_element_spacing_m
        bint multipath_enabled
        double surface_z
        double surface_reflectivity
        double surface_rms_height_m
        Vec3 platform_velocity_mps
        double platform_yaw_rate_dps
        double sweep_duration_s
        int motion_samples_per_bin
        int num_threads
    cdef cppclass Pose:
        Pose()
        Vec3 position
        Vec3 x_axis
        Vec3 y_axis
        Vec3 z_axis
    void render(const SonarConfig&, const Scene&, const Pose&, double*) nogil
    void apply_speckle(double*, int, unsigned int) nogil


cdef extern from "waveform.h":
    cdef cppclass ChirpConfig:
        ChirpConfig()
        double frequency_hz
        double chirp_bandwidth_hz
        double chirp_duration_s
        double sample_rate_hz
        double speed_of_sound_mps
    cdef cppclass Reflector:
        Reflector()
        double range_m
        double reflectivity
        double cos_incidence
    int waveform_sample_count(const ChirpConfig&)
    void generate_chirp(const ChirpConfig&, double*, int) nogil
    void generate_tone(const ChirpConfig&, double*, int) nogil
    void receive_echoes(const ChirpConfig&, const Reflector*, int, const double*, int,
                        double*, int) nogil
    void add_gaussian_noise(double*, int, double, unsigned int) nogil
    void matched_filter(const double*, int, const double*, int, double*, int) nogil


cdef extern from "camera.h":
    cdef cppclass Camera:
        Camera()
        Pose pose
        int width
        int height
        double focal_px
        double attenuation_per_m
        double veiling_radiance
        double light_intensity
    bint project_point(const Camera&, const Vec3&, double*, double*)
    Vec3 camera_ray(const Camera&, double, double)
    void render_camera(const Camera&, const Scene&, double*) nogil


cdef Vec3 _vec(values):
    cdef Vec3 v
    v.x = values[0]; v.y = values[1]; v.z = values[2]
    return v


cdef Texture _texture(item):
    cdef Texture t
    t.amplitude = item.get("texture_amplitude", 0.0)
    t.scale_m = item.get("texture_scale_m", 0.25)
    t.seed = item.get("texture_seed", 1)
    return t


cdef Scene _build_scene(objects) except *:
    cdef Scene scene
    cdef Plane plane
    cdef Sphere sphere
    cdef Cylinder cylinder

    for item in objects:
        kind = item["kind"]
        if kind == "plane":
            plane.point = _vec(item["point"])
            plane.normal = _vec(item["normal"])
            plane.reflectivity = item["reflectivity"]
            plane.texture = _texture(item)
            scene.planes.push_back(plane)
        elif kind == "sphere":
            sphere.centre = _vec(item["centre"])
            sphere.radius = item["radius"]
            sphere.reflectivity = item["reflectivity"]
            sphere.texture = _texture(item)
            scene.spheres.push_back(sphere)
        elif kind == "cylinder":
            cylinder.centre = _vec(item["centre"])
            cylinder.axis = _vec(item["axis"])
            cylinder.radius = item["radius"]
            cylinder.half_length = item["half_length"]
            cylinder.reflectivity = item["reflectivity"]
            cylinder.texture = _texture(item)
            scene.cylinders.push_back(cylinder)
        else:
            raise ValueError("unknown object kind: %r" % (kind,))
    return scene


cdef _apply_axes(Pose* pose, axes):
    if axes is None:
        return
    a = np.asarray(axes, dtype=np.float64)
    pose.x_axis = _vec(a[:, 0]); pose.y_axis = _vec(a[:, 1]); pose.z_axis = _vec(a[:, 2])


def array_beam_pattern(double phi_rad, int element_count=64,
                       double element_spacing_m=0.0, double wavelength_m=1.0):
    """B(phi), the squared array factor. Spacing of zero means half a wavelength."""
    if element_spacing_m <= 0.0:
        element_spacing_m = 0.5 * wavelength_m
    return beam_pattern(phi_rad, element_count, element_spacing_m, wavelength_m)


def thorp_alpha(double frequency_hz):
    """Thorp absorption in dB per km."""
    return thorp_absorption_db_per_km(frequency_hz)


def two_way_loss_db(double alpha_db_per_km, double range_m):
    return transmission_loss_db(alpha_db_per_km, range_m)


def surface_texture(point, double amplitude, double scale_m=0.25, unsigned int seed=1):
    """Backscatter multiplier at a world point. Mean 1, deterministic in the point."""
    cdef Texture t
    t.amplitude = amplitude; t.scale_m = scale_m; t.seed = seed
    return texture_factor(_vec(point), t)


def ray_plane(point, normal, reflectivity, origin, direction):
    """Distance to a plane along a unit direction, or None if it is not hit."""
    cdef Plane p
    p.point = _vec(point); p.normal = _vec(normal); p.reflectivity = reflectivity
    cdef Hit h = intersect_plane(p, _vec(origin), _vec(direction))
    return h.t if h.valid else None


def ray_sphere(centre, double radius, reflectivity, origin, direction):
    cdef Sphere s
    s.centre = _vec(centre); s.radius = radius; s.reflectivity = reflectivity
    cdef Hit h = intersect_sphere(s, _vec(origin), _vec(direction))
    return h.t if h.valid else None


def ray_cylinder(centre, axis, double radius, double half_length, reflectivity,
                 origin, direction):
    cdef Cylinder c
    c.centre = _vec(centre); c.axis = _vec(axis)
    c.radius = radius; c.half_length = half_length; c.reflectivity = reflectivity
    cdef Hit h = intersect_cylinder(c, _vec(origin), _vec(direction))
    return h.t if h.valid else None


def make_plane(point, normal, reflectivity=0.05, texture_amplitude=0.0,
               texture_scale_m=0.25, texture_seed=1):
    return {"kind": "plane", "point": tuple(point), "normal": tuple(normal),
            "reflectivity": float(reflectivity),
            "texture_amplitude": float(texture_amplitude),
            "texture_scale_m": float(texture_scale_m), "texture_seed": int(texture_seed)}


def make_sphere(centre, radius, reflectivity=0.5, texture_amplitude=0.0,
                texture_scale_m=0.25, texture_seed=1):
    return {"kind": "sphere", "centre": tuple(centre), "radius": float(radius),
            "reflectivity": float(reflectivity),
            "texture_amplitude": float(texture_amplitude),
            "texture_scale_m": float(texture_scale_m), "texture_seed": int(texture_seed)}


def make_cylinder(centre, axis, radius, half_length, reflectivity=0.8,
                  texture_amplitude=0.0, texture_scale_m=0.25, texture_seed=1):
    """A capped finite cylinder: the pipe, mine or debris stand-in."""
    return {"kind": "cylinder", "centre": tuple(centre), "axis": tuple(axis),
            "radius": float(radius), "half_length": float(half_length),
            "reflectivity": float(reflectivity),
            "texture_amplitude": float(texture_amplitude),
            "texture_scale_m": float(texture_scale_m), "texture_seed": int(texture_seed)}


cdef class SonarSimulator:
    """A forward-scan imaging sonar."""

    cdef SonarConfig cfg

    def __init__(self, double frequency_hz=1.8e6, int num_azimuth_bins=256,
                 int num_range_bins=512, double horizontal_fov_deg=30.0,
                 double vertical_beamwidth_deg=14.0, double max_range_m=10.0,
                 double speed_of_sound_mps=1500.0, int num_elevation_subrays=48,
                 int array_element_count=64, double array_element_spacing_m=0.0,
                 bint multipath_enabled=False, double surface_z=0.0,
                 double surface_reflectivity=1.0, double surface_rms_height_m=0.0,
                 platform_velocity_mps=(0.0, 0.0, 0.0),
                 double platform_yaw_rate_dps=0.0, double sweep_duration_s=0.0,
                 int motion_samples_per_bin=1, int num_threads=0):
        self.cfg.frequency_hz = frequency_hz
        self.cfg.num_azimuth_bins = num_azimuth_bins
        self.cfg.num_range_bins = num_range_bins
        self.cfg.horizontal_fov_deg = horizontal_fov_deg
        self.cfg.vertical_beamwidth_deg = vertical_beamwidth_deg
        self.cfg.max_range_m = max_range_m
        self.cfg.speed_of_sound_mps = speed_of_sound_mps
        self.cfg.num_elevation_subrays = num_elevation_subrays
        self.cfg.array_element_count = array_element_count
        self.cfg.array_element_spacing_m = array_element_spacing_m
        self.cfg.multipath_enabled = multipath_enabled
        self.cfg.surface_z = surface_z
        self.cfg.surface_reflectivity = surface_reflectivity
        self.cfg.surface_rms_height_m = surface_rms_height_m
        self.cfg.platform_velocity_mps = _vec(platform_velocity_mps)
        self.cfg.platform_yaw_rate_dps = platform_yaw_rate_dps
        self.cfg.sweep_duration_s = sweep_duration_s
        self.cfg.motion_samples_per_bin = motion_samples_per_bin
        self.cfg.num_threads = num_threads

    @property
    def wavelength_m(self):
        return self.cfg.speed_of_sound_mps / self.cfg.frequency_hz

    @property
    def range_resolution_m(self):
        return self.cfg.max_range_m / self.cfg.num_range_bins

    @property
    def multipath(self):
        return bool(self.cfg.multipath_enabled)

    @multipath.setter
    def multipath(self, bint value):
        self.cfg.multipath_enabled = value

    @property
    def surface_rms_height_m(self):
        return self.cfg.surface_rms_height_m

    @surface_rms_height_m.setter
    def surface_rms_height_m(self, double value):
        self.cfg.surface_rms_height_m = value

    @property
    def num_elevation_subrays(self):
        return self.cfg.num_elevation_subrays

    @num_elevation_subrays.setter
    def num_elevation_subrays(self, int value):
        self.cfg.num_elevation_subrays = value

    @property
    def sweep_duration_s(self):
        return self.cfg.sweep_duration_s

    @sweep_duration_s.setter
    def sweep_duration_s(self, double value):
        self.cfg.sweep_duration_s = value

    @property
    def motion_samples_per_bin(self):
        return self.cfg.motion_samples_per_bin

    @motion_samples_per_bin.setter
    def motion_samples_per_bin(self, int value):
        self.cfg.motion_samples_per_bin = value

    @property
    def platform_yaw_rate_dps(self):
        return self.cfg.platform_yaw_rate_dps

    @platform_yaw_rate_dps.setter
    def platform_yaw_rate_dps(self, double value):
        self.cfg.platform_yaw_rate_dps = value

    @property
    def platform_velocity_mps(self):
        return (self.cfg.platform_velocity_mps.x, self.cfg.platform_velocity_mps.y,
                self.cfg.platform_velocity_mps.z)

    @platform_velocity_mps.setter
    def platform_velocity_mps(self, value):
        self.cfg.platform_velocity_mps = _vec(value)

    @property
    def shape(self):
        return (self.cfg.num_azimuth_bins, self.cfg.num_range_bins)

    def azimuth_axis_deg(self):
        half = self.cfg.horizontal_fov_deg / 2.0
        step = self.cfg.horizontal_fov_deg / self.cfg.num_azimuth_bins
        return -half + step * (np.arange(self.cfg.num_azimuth_bins) + 0.5)

    def range_axis_m(self):
        step = self.cfg.max_range_m / self.cfg.num_range_bins
        return step * (np.arange(self.cfg.num_range_bins) + 0.5)

    def sweep_time_s(self):
        """Instant each bearing bin is formed, which is what the smear follows."""
        step = self.cfg.sweep_duration_s / self.cfg.num_azimuth_bins
        return step * (np.arange(self.cfg.num_azimuth_bins) + 0.5)

    def render(self, objects, position=(0.0, 0.0, 0.0), axes=None,
               bint speckle=False, unsigned int seed=0):
        """Render a scene. axes columns are (X_s, Y_s, Z_s); default is identity."""
        cdef Scene scene = _build_scene(objects)

        cdef Pose pose
        pose.position = _vec(position)
        _apply_axes(&pose, axes)

        out = np.zeros((self.cfg.num_azimuth_bins, self.cfg.num_range_bins), dtype=np.float64)
        cdef double[:, ::1] view = out
        cdef int count = self.cfg.num_azimuth_bins * self.cfg.num_range_bins
        cdef bint do_speckle = speckle
        cdef unsigned int s = seed
        with nogil:
            render(self.cfg, scene, pose, &view[0, 0])
            if do_speckle:
                apply_speckle(&view[0, 0], count, s)
        return out


cdef class OpticalCamera:
    """A pinhole camera in the same world, with the same frame convention.

    The point of it is the elevation the sonar cannot measure: a pixel fixes a
    bearing and an elevation but no range, a sonar bin fixes a range and a
    bearing but no elevation, and between them the point is determined.
    """

    cdef Camera cam

    def __init__(self, int width=320, int height=240, double focal_px=260.0,
                 position=(0.0, 0.0, 0.0), axes=None, double attenuation_per_m=0.0,
                 double veiling_radiance=0.35, double light_intensity=20.0):
        self.cam.width = width
        self.cam.height = height
        self.cam.focal_px = focal_px
        self.cam.attenuation_per_m = attenuation_per_m
        self.cam.veiling_radiance = veiling_radiance
        self.cam.light_intensity = light_intensity
        self.cam.pose.position = _vec(position)
        _apply_axes(&self.cam.pose, axes)

    @property
    def shape(self):
        return (self.cam.height, self.cam.width)

    @property
    def focal_px(self):
        return self.cam.focal_px

    @property
    def position(self):
        return (self.cam.pose.position.x, self.cam.pose.position.y, self.cam.pose.position.z)

    @property
    def attenuation_per_m(self):
        return self.cam.attenuation_per_m

    @attenuation_per_m.setter
    def attenuation_per_m(self, double value):
        self.cam.attenuation_per_m = value

    def horizontal_fov_deg(self):
        return 2.0 * np.degrees(np.arctan(0.5 * self.cam.width / self.cam.focal_px))

    def project(self, point):
        """Pixel (u, v) for a world point, or None if it is behind the camera."""
        cdef double u = 0.0, v = 0.0
        if not project_point(self.cam, _vec(point), &u, &v):
            return None
        return (u, v)

    def ray(self, double u, double v):
        cdef Vec3 d = camera_ray(self.cam, u, v)
        return (d.x, d.y, d.z)

    def render(self, objects):
        cdef Scene scene = _build_scene(objects)
        out = np.zeros((self.cam.height, self.cam.width), dtype=np.float64)
        cdef double[:, ::1] view = out
        with nogil:
            render_camera(self.cam, scene, &view[0, 0])
        return out


cdef class ChirpSonar:
    """Time-domain pulse-compression sonar.

    Separate from SonarSimulator on purpose: this one produces waveforms, not
    images. The two share the intensity model in physics.h, so a reflector at a
    given range is scaled the same way in both, and the matched-filter peak comes
    out proportional to the square root of the imaging pipeline's intensity.
    """

    cdef ChirpConfig cfg
    cdef int threads

    def __init__(self, double frequency_hz=300e3, double chirp_bandwidth_hz=60e3,
                 double chirp_duration_s=2e-3, double sample_rate_hz=1.2e6,
                 double speed_of_sound_mps=1500.0, int num_threads=0):
        if sample_rate_hz < 2.0 * (frequency_hz + chirp_bandwidth_hz):
            raise ValueError("sample rate is below Nyquist for f0 + B")
        self.cfg.frequency_hz = frequency_hz
        self.cfg.chirp_bandwidth_hz = chirp_bandwidth_hz
        self.cfg.chirp_duration_s = chirp_duration_s
        self.cfg.sample_rate_hz = sample_rate_hz
        self.cfg.speed_of_sound_mps = speed_of_sound_mps
        self.threads = num_threads

    @property
    def sample_rate_hz(self):
        return self.cfg.sample_rate_hz

    @property
    def range_resolution_m(self):
        """delta_r = c / (2 B), the compressed resolution."""
        return self.cfg.speed_of_sound_mps / (2.0 * self.cfg.chirp_bandwidth_hz)

    @property
    def uncompressed_resolution_m(self):
        """c T / 2, what a plain pulse of the same duration manages."""
        return 0.5 * self.cfg.speed_of_sound_mps * self.cfg.chirp_duration_s

    @property
    def time_bandwidth_product(self):
        return self.cfg.chirp_bandwidth_hz * self.cfg.chirp_duration_s

    def range_axis_m(self, int count):
        """Range each sample of a record corresponds to, r = c n / (2 fs)."""
        return (0.5 * self.cfg.speed_of_sound_mps / self.cfg.sample_rate_hz) * np.arange(count)

    def transmit(self, bint swept=True):
        """The transmitted waveform: the chirp, or the plain tone it is measured against."""
        count = waveform_sample_count(self.cfg)
        out = np.zeros(count, dtype=np.float64)
        cdef double[::1] view = out
        cdef int n = count
        with nogil:
            if swept:
                generate_chirp(self.cfg, &view[0], n)
            else:
                generate_tone(self.cfg, &view[0], n)
        return out

    def receive(self, reflectors, transmit, double record_s):
        """Sum the transmitted waveform delayed and scaled by every reflector.

        reflectors: sequence of (range_m, reflectivity, cos_incidence).
        """
        cdef vector[Reflector] targets
        cdef Reflector target
        for item in reflectors:
            target.range_m = item[0]
            target.reflectivity = item[1]
            target.cos_incidence = item[2] if len(item) > 2 else 1.0
            targets.push_back(target)

        sent = np.ascontiguousarray(transmit, dtype=np.float64)
        out = np.zeros(int(record_s * self.cfg.sample_rate_hz), dtype=np.float64)
        cdef double[::1] sent_view = sent
        cdef double[::1] out_view = out
        cdef int sent_count = sent.shape[0]
        cdef int out_count = out.shape[0]
        cdef int target_count = targets.size()
        with nogil:
            receive_echoes(self.cfg, targets.data(), target_count, &sent_view[0], sent_count,
                           &out_view[0], out_count)
        return out

    def add_noise(self, signal, double sigma, unsigned int seed=0):
        out = np.array(signal, dtype=np.float64, copy=True)
        cdef double[::1] view = out
        cdef int count = out.shape[0]
        cdef double s = sigma
        cdef unsigned int k = seed
        with nogil:
            add_gaussian_noise(&view[0], count, s, k)
        return out

    def compress(self, received, reference):
        """Matched filter: correlate the record against the transmitted copy."""
        record = np.ascontiguousarray(received, dtype=np.float64)
        copy = np.ascontiguousarray(reference, dtype=np.float64)
        out = np.zeros(record.shape[0], dtype=np.float64)
        cdef double[::1] record_view = record
        cdef double[::1] copy_view = copy
        cdef double[::1] out_view = out
        cdef int record_count = record.shape[0]
        cdef int copy_count = copy.shape[0]
        cdef int workers = self.threads
        with nogil:
            matched_filter(&record_view[0], record_count, &copy_view[0], copy_count,
                           &out_view[0], workers)
        return out
