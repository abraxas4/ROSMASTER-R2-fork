-- Same as r2_cartographer_fused.lua + depth camera scan (depthimage_to_laserscan) as 2nd laser.
-- Topics become scan_1 (LiDAR /scan) and scan_2 (/depth_scan); launch remaps them.
include "r2_cartographer_fused.lua"

options.num_laser_scans = 2

return options
