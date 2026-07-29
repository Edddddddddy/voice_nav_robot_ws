# Can generate and validate custom ROS interfaces

The learner hand-wrote a nested MissionStep message and ExecuteMission action, connected them to rosidl, inspected the generated Interface, and corrected invalid package dependencies after using rosdep as an additional validation surface. They also identified that motion limits and timeouts belong to trusted runtime policy rather than LLM output, so future lessons can treat the LLM as an untrusted planner.
