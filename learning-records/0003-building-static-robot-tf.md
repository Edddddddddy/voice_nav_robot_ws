# Can build and verify a static robot model and TF tree

The learner hand-wrote a Xacro differential-drive robot, a display launch file, and the required package installation rules. They validated the generated URDF, inspected the model in RViz, verified `base_footprint -> laser_link` numerically, and corrected the important misconception that a joint origin is relative to its parent frame rather than the ground. All 12 `voice_nav_sim` tests pass, so future lessons can assume basic URDF link/joint, TF, Xacro macro, launch substitution, and package dependency fluency.
