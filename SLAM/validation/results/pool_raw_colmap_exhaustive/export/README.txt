Pool reconstruction exported from pool_raw_colmap_exhaustive (all 117 frames registered).

Units: reconstruction units (monocular: metric scale unknown).
Frame: z: height above the fitted pool floor plane; x: along the floor lane stripes; origin: floor point under the centre of the camera path. x_pixel ~ K (R X + t), C = -R' t. Pixel coordinates put the centre of the top left pixel at (0, 0) (OpenCV); in MATLAB, image(v + 1, u + 1) is the pixel at (u, v).

pool_reconstruction.mat (MATLAB: load('pool_reconstruction.mat')):
  K            3x3 camera matrix used (fixed, not refined)
  image_names  file name of each frame (opt<n>.bmp); frame_numbers the n
  Rt           3x4xN, [R | t] of each frame, same layout as Final_Proj in OSCalibration.mat
  P            3x4xN, K * Rt
  C            Nx3 camera centres: the estimated camera trajectory
  points       Mx3 3D model points; colors Mx3 (RGB 0..255)
camera_trajectory.csv: the same cameras, one row per frame (R row major).
model.ply: the 3D model (MeshLab, CloudCompare).

Camera height above the floor: median 0.973; median height of the model points above the floor: 0.013 (most points lie on the floor and the pebble mat).
6.8 % of the points lie above the cameras: the lane ropes at the surface and features seen in reflections in the underside of the moving water surface. The reflection points are not physical structure; they were left in rather than removed by hand.
