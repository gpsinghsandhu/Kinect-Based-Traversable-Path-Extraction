Kinect-Based-Traversable-Path-Extraction
========================================

This project is implementation of traversable path extraction for a differential drive robot with kinect as the main sensor.
This project was developed as a preliminary path extraction software for indoor mobile robot. A part of research paper by Joydeep Biswas and Manuela Veloso - "Depth Camera Based Indoor Mobile Robot Localization and Navigation" was implemented.

The program basically uses FSPF (Fast Sample Plane Filtering) to filter out bulk of the redundant  3D points from kinect (points belonging to same plane do not necessarily give useful info) which is then used in a very simple threshold procedure to find traversable and non-traversable path.
The threshold procedure can be made more extensive and robot-specific for actual implementation on the robot.
