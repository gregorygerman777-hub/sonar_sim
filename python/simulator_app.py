"""Forward-scan sonar console.

A sonar operator's display, not a plot of a sonar: a bearing trace over the
beam-bin image, an A-scan and a camera trace, and two time records that fill as
the platform runs. Every frame is rendered by the C++ core through the Cython
bindings, so what is on screen is the simulator itself.

    .venv/bin/python python/simulator_app.py

Three things keep it responsive. The core threads over bearing. The images are
converted to 8-bit RGB here with a lookup table, so matplotlib is handed pixels
rather than being asked to normalise and colour-map floats every frame. And
everything that changes is blitted over a cached background, so nine sliders and
six sets of axis furniture are drawn once rather than thirty times a second.
"""

import sys
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons, Slider

sys.path.insert(0, ".")
sys.path.insert(0, "python")
import scene as sc
import sonar

DRAFT_SUBRAYS, FULL_SUBRAYS = 192, 1280
N_AZIMUTH, N_RANGE, MAX_RANGE = 256, 512, 10.0
FOV_DEG = 30.0
HISTORY = 150                      # rows in each time record
FLOOR_Z, SONAR_Z = -2.6, -0.6
CAMERA_RISE = 0.25
SWEEP_S = 0.35
FLOOR_DB = -62.0
BTR_FLOOR = -26.0   # the bearing record spans far less than the image does

INK = "#000000"
PHOSPHOR = "#00ff40"
TRACE = "#ffff00"
MARK = "#ff2b20"
COMPASS = "#ff2b20"
RELATIVE = "#4f7bff"
FRAME = "#8a1410"
DIM = "#7a8a80"


def phosphor_table():
    """Black through saturated green to a white-hot tip, the way a CRT reads.

    Four channels, not three. Handed an RGB array, matplotlib builds an RGBA one
    and multiplies the alpha through a masked array every frame; handing it RGBA
    already skips that.
    """
    level = np.linspace(0.0, 1.0, 256)
    green = level ** 0.62
    red = np.clip((level - 0.70) / 0.30, 0.0, 1.0) ** 1.5
    blue = np.clip((level - 0.82) / 0.18, 0.0, 1.0) ** 1.8
    opaque = np.ones_like(level)
    return (np.stack([red, green, blue, opaque], axis=1) * 255.0).astype(np.uint8)


LUT = phosphor_table()


def to_pixels(decibels, floor=FLOOR_DB):
    """dB to 8-bit RGBA through the lookup table, skipping matplotlib's colour map."""
    index = np.clip((decibels - floor) * (255.0 / -floor), 0.0, 255.0).astype(np.uint8)
    return LUT[index]


def build_scene(tilt_deg, target_range_m, texture, cylinder):
    tilt = np.radians(tilt_deg)
    centre = np.array([0.0, target_range_m * np.cos(tilt),
                       SONAR_Z - target_range_m * np.sin(tilt)])
    floor = sonar.make_plane((0.0, 0.0, FLOOR_Z), (0.0, 0.0, 1.0), 0.06,
                             texture_amplitude=texture, texture_scale_m=0.30)
    if cylinder:
        yaw = np.radians(35.0)
        target = sonar.make_cylinder(centre, (np.cos(yaw), np.sin(yaw), 0.0), 0.13, 0.55,
                                     0.9, texture_amplitude=0.4 * texture,
                                     texture_scale_m=0.10)
    else:
        target = sonar.make_sphere(centre, 0.30, 0.9, texture_amplitude=0.4 * texture,
                                   texture_scale_m=0.10)
    return [floor, target], centre


fig = plt.figure(figsize=(15.6, 9.6))
fig.patch.set_facecolor(INK)

power_ax = fig.add_axes([0.04, 0.905, 0.93, 0.072])
scale_ax = fig.add_axes([0.04, 0.856, 0.93, 0.046])
main_ax = fig.add_axes([0.04, 0.545, 0.93, 0.305])
ascan_ax = fig.add_axes([0.04, 0.462, 0.455, 0.066])
camera_trace_ax = fig.add_axes([0.515, 0.462, 0.455, 0.066])
btr_ax = fig.add_axes([0.04, 0.235, 0.455, 0.222])
rtr_ax = fig.add_axes([0.515, 0.235, 0.455, 0.222])

for ax in (power_ax, main_ax, ascan_ax, camera_trace_ax, btr_ax, rtr_ax):
    ax.set_facecolor(INK)
    for spine in ax.spines.values():
        spine.set_color(FRAME)
    ax.tick_params(colors=COMPASS, labelsize=7, length=2)

for ax in (power_ax, ascan_ax, camera_trace_ax):
    ax.set_xticks([])
    ax.set_yticks([])
scale_ax.set_axis_off()

azimuth_axis = -0.5 * FOV_DEG + FOV_DEG * (np.arange(N_AZIMUTH) + 0.5) / N_AZIMUTH
range_axis = MAX_RANGE / N_RANGE * (np.arange(N_RANGE) + 0.5)

# Top strip: total energy against bearing, with the reference line the reference
# console draws across the top and a cursor on the selected bearing.
(power_line,) = power_ax.plot(azimuth_axis, np.zeros(N_AZIMUTH), color=TRACE, linewidth=1.1)
power_ax.axhline(1.0, color="#ffffff", linestyle=(0, (6, 4)), linewidth=0.8)
power_cursor = power_ax.axvline(0.0, color=TRACE, linewidth=1.4)
power_ax.set(xlim=(azimuth_axis[0], azimuth_axis[-1]), ylim=(0.0, 1.12))
power_ax.add_patch(plt.Rectangle((azimuth_axis[0] + 0.2, 0.18), 0.5, 0.72,
                                 color=TRACE, transform=power_ax.transData))

# Two scales, as on the reference: true bearing in red, relative in blue.
TICKS = np.linspace(-0.5 * FOV_DEG, 0.5 * FOV_DEG, 11)
compass_labels, relative_labels = [], []
for tick in TICKS:
    fraction = (tick + 0.5 * FOV_DEG) / FOV_DEG
    compass_labels.append(scale_ax.text(fraction, 0.62, "", color=COMPASS, fontsize=8,
                                        ha="center", family="monospace",
                                        transform=scale_ax.transAxes))
    relative_labels.append(scale_ax.text(fraction, 0.06, f"{tick:+.0f}", color=RELATIVE,
                                         fontsize=8, ha="center", family="monospace",
                                         transform=scale_ax.transAxes))

# Main display: bearing across, range downward, which is the reference layout and
# also the way a forward-scan image is usually flown.
main_artist = main_ax.imshow(np.zeros((N_RANGE, N_AZIMUTH, 4), dtype=np.uint8),
                             aspect="auto", interpolation="nearest",
                             extent=[azimuth_axis[0], azimuth_axis[-1], -MAX_RANGE, 0.0])
(detections,) = main_ax.plot([], [], linestyle="none", marker="s", markersize=1.8,
                             color=MARK)
main_ax.set_xticks(TICKS)
main_ax.set_yticks(-np.arange(0, int(MAX_RANGE) + 1, 2))
main_ax.tick_params(labelcolor=COMPASS)

(ascan_line,) = ascan_ax.plot(range_axis, np.zeros(N_RANGE), color=TRACE, linewidth=0.9)
ascan_ax.set(xlim=(0.0, MAX_RANGE), ylim=(FLOOR_DB, 4.0))
ascan_label = ascan_ax.text(0.008, 0.72, "A-SCAN", color=MARK, fontsize=8,
                            family="monospace", transform=ascan_ax.transAxes)

(camera_line,) = camera_trace_ax.plot(np.arange(160), np.zeros(160), color=TRACE,
                                      linewidth=0.9)
camera_trace_ax.set(xlim=(0, 159), ylim=(0.0, 1.05))
camera_label = camera_trace_ax.text(0.008, 0.72, "OPTICAL", color=MARK, fontsize=8,
                                    family="monospace", transform=camera_trace_ax.transAxes)

# Two time records. New rows enter at the top and age downward, so the vertical
# axis is time before now, exactly as on the reference display.
btr_history = np.full((HISTORY, N_AZIMUTH), BTR_FLOOR)
rtr_history = np.full((HISTORY, N_RANGE), FLOOR_DB)
btr_artist = btr_ax.imshow(np.zeros((HISTORY, N_AZIMUTH, 4), dtype=np.uint8),
                           aspect="auto", interpolation="nearest",
                           extent=[azimuth_axis[0], azimuth_axis[-1], -HISTORY, 0])
rtr_artist = rtr_ax.imshow(np.zeros((HISTORY, N_RANGE, 4), dtype=np.uint8),
                           aspect="auto", interpolation="nearest",
                           extent=[0.0, MAX_RANGE, -HISTORY, 0])
btr_ax.set(xticks=TICKS[::2], yticks=-np.arange(0, HISTORY + 1, 30))
rtr_ax.set(xticks=np.arange(0, int(MAX_RANGE) + 1, 2), yticks=-np.arange(0, HISTORY + 1, 30))
for ax, text in ((btr_ax, "BEARING-TIME"), (rtr_ax, "RANGE-TIME")):
    ax.text(0.008, 0.955, text, color=MARK, fontsize=8, family="monospace",
            va="top", transform=ax.transAxes)

controls = {}
specs = [("tilt", 0.0, 35.0, 16.0, 0), ("roll", 0.0, 90.0, 0.0, 0),
         ("beamwidth", 4.0, 30.0, 12.0, 0), ("freq kHz", 100.0, 2000.0, 900.0, 0),
         ("range m", 2.0, 8.0, 4.5, 0), ("heading", 0.0, 359.0, 348.0, 0),
         ("wave mm", 0.0, 2.0, 0.0, 1), ("turbidity", 0.02, 1.2, 0.10, 1),
         ("texture", 0.0, 0.9, 0.45, 1), ("surge m/s", 0.0, 2.0, 0.0, 1),
         ("yaw deg/s", -40.0, 40.0, 0.0, 1)]
rows = [0, 0]
for label, low, high, start, column in specs:
    ax = fig.add_axes([0.085 + 0.375 * column, 0.185 - 0.030 * rows[column], 0.235, 0.017])
    rows[column] += 1
    ax.set_facecolor("#0d1a0d")
    controls[label] = Slider(ax, label, low, high, valinit=start, color=PHOSPHOR)
    controls[label].label.set_color(COMPASS)
    controls[label].label.set_fontsize(8)
    controls[label].label.set_family("monospace")
    controls[label].valtext.set_color(TRACE)
    controls[label].valtext.set_fontsize(8)
    controls[label].valtext.set_family("monospace")

toggle_ax = fig.add_axes([0.80, 0.035, 0.17, 0.165])
toggle_ax.set_facecolor("#0d1a0d")
for spine in toggle_ax.spines.values():
    spine.set_color(FRAME)
toggles = CheckButtons(toggle_ax, ["multipath", "speckle", "cylinder", "run"],
                       [True, True, False, False])
for label in toggles.labels:
    label.set_color(PHOSPHOR)
    label.set_fontsize(9)
    label.set_family("monospace")

banner = fig.text(0.04, 0.984, "", color=PHOSPHOR, fontsize=10, family="monospace")
status = fig.text(0.60, 0.984, "", color=DIM, fontsize=9, family="monospace")

state = {"advance": 0.0, "frame": 0, "ms": 0.0}


def render(subrays):
    started = time.perf_counter()
    tilt = controls["tilt"].val
    frequency_khz = controls["freq kHz"].val
    yaw_rate = controls["yaw deg/s"].val
    multipath, speckle, cylinder, _ = toggles.get_status()

    objects, centre = build_scene(tilt, controls["range m"].val, controls["texture"].val,
                                  cylinder)
    axes = sc.tilted_axes(tilt, controls["roll"].val)
    position = np.array([0.0, 0.0, SONAR_Z]) + state["advance"] * axes[:, 1]

    sim = sonar.SonarSimulator(frequency_hz=frequency_khz * 1e3, num_azimuth_bins=N_AZIMUTH,
                               num_range_bins=N_RANGE, horizontal_fov_deg=FOV_DEG,
                               vertical_beamwidth_deg=controls["beamwidth"].val,
                               max_range_m=MAX_RANGE, num_elevation_subrays=subrays,
                               multipath_enabled=multipath, surface_z=0.0,
                               surface_rms_height_m=controls["wave mm"].val * 1e-3,
                               platform_yaw_rate_dps=yaw_rate,
                               sweep_duration_s=SWEEP_S if yaw_rate else 0.0)
    image = sim.render(objects, position=position, axes=axes, speckle=speckle,
                       seed=state["frame"])

    camera = sonar.OpticalCamera(width=160, height=120, focal_px=125.0,
                                 position=position + np.array([0.0, 0.0, CAMERA_RISE]),
                                 axes=axes, attenuation_per_m=controls["turbidity"].val,
                                 light_intensity=150.0)
    frame = camera.render(objects)

    state["ms"] = 1000 * (time.perf_counter() - started)
    return sim, image, frame, position, centre


def refresh(subrays):
    sim, image, frame, position, centre = render(subrays)
    decibels = sc.to_decibels(image, floor_db=FLOOR_DB)

    # Main panel: range downward means the array is transposed, bearing across.
    main_artist.set_data(to_pixels(decibels.T))

    # Detections: the strongest bin on each bearing, where it stands clear of the
    # noise. This is the console's red tracker marks, not a separate algorithm.
    peak_bins = np.argmax(decibels, axis=1)
    peak_values = decibels[np.arange(N_AZIMUTH), peak_bins]
    lit = peak_values > FLOOR_DB + 22.0
    detections.set_data(azimuth_axis[lit], -range_axis[peak_bins[lit]])

    energy_db = sc.to_decibels(image.sum(axis=1), floor_db=FLOOR_DB)
    power_line.set_ydata(1.0 + energy_db / -FLOOR_DB)
    power_cursor.set_xdata([0.0, 0.0])

    centre_beam = sc.to_decibels(image[N_AZIMUTH // 2], floor_db=FLOOR_DB,
                                reference=image.max())
    ascan_line.set_ydata(centre_beam)

    row = frame[frame.shape[0] // 2]
    camera_line.set_ydata(row / max(row.max(), 1e-9))

    btr_history[1:] = btr_history[:-1]
    btr_history[0] = energy_db
    rtr_history[1:] = rtr_history[:-1]
    rtr_history[0] = centre_beam
    btr_artist.set_data(to_pixels(btr_history, floor=BTR_FLOOR))
    rtr_artist.set_data(to_pixels(rtr_history))

    heading = controls["heading"].val
    for label, tick in zip(compass_labels, TICKS):
        label.set_text(f"{(heading + tick) % 360:03.0f}")

    alpha = sonar.thorp_alpha(controls["freq kHz"].val * 1e3)
    r_direct = float(np.linalg.norm(centre - position))
    banner.set_text(f"FSS  {controls['freq kHz'].val:.0f} kHz  "
                    f"lambda {1000 * sim.wavelength_m:.2f} mm  "
                    f"alpha {alpha:.0f} dB/km  target {r_direct:.2f} m  "
                    f"advance {state['advance']:.2f} m")
    status.set_text(f"{subrays:4d} sub-rays  {N_AZIMUTH}x{N_RANGE}  "
                    f"{state['ms']:5.1f} ms core  "
                    f"{'FULL' if subrays == FULL_SUBRAYS else 'DRAFT'}")
    blit()


dynamic = [main_artist, detections, power_line, power_cursor, ascan_line, camera_line,
           btr_artist, rtr_artist, banner, status, ascan_label, camera_label]
dynamic += compass_labels
for slider in controls.values():
    slider.drawon = False
    dynamic += [slider.poly, slider._handle, slider.valtext]
for artist in dynamic:
    artist.set_animated(True)

background = None


def draw_dynamic():
    for artist in dynamic:
        (artist.axes or fig).draw_artist(artist)


def on_draw(_event=None):
    global background
    background = fig.canvas.copy_from_bbox(fig.bbox)
    draw_dynamic()


def blit():
    if background is None:
        fig.canvas.draw()
        return
    fig.canvas.restore_region(background)
    draw_dynamic()
    fig.canvas.blit(fig.bbox)


def on_change(_=None):
    refresh(DRAFT_SUBRAYS)


def on_release(_event):
    if status.get_text().endswith("DRAFT"):
        refresh(FULL_SUBRAYS)


def tick(_event=None):
    """One console update. Running advances the platform, so the time records
    fill with a real track rather than a repeated row."""
    state["frame"] += 1
    if toggles.get_status()[3]:
        state["advance"] += controls["surge m/s"].val * 0.05
        if state["advance"] > controls["range m"].val - 0.8:
            state["advance"] = 0.0
        refresh(DRAFT_SUBRAYS)


for slider in controls.values():
    slider.on_changed(on_change)
toggles.on_clicked(on_change)
fig.canvas.mpl_connect("draw_event", on_draw)
fig.canvas.mpl_connect("button_release_event", on_release)

if __name__ == "__main__":
    timer = fig.canvas.new_timer(interval=50)
    timer.add_callback(tick)
    timer.start()
    refresh(FULL_SUBRAYS)
    plt.show()
else:
    refresh(FULL_SUBRAYS)
