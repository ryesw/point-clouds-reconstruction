# point-clouds-reconstruction
Reconstruction global point clouds using partial point clouds

## Process
1. Load global dataset
2. Initialize empty point clouds to reconstruction
3. For loop
 • Pose estimation from dataset (Feature Descriptos)
 • Ex) data[0] and data[1], data[1] and data[2], ..., data[28] and data[29]
 • Ex) Get 𝑇_1_0, T_2_1, T_3_2, ..., T_29_28
4. Move all point clouds to the same coordinate system.
5. Save point clouds to PLY fil
