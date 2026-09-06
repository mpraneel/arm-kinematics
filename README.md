# arm-kinematics
Planar robot arm kinematics in C++17 with a runtime supervisor. Analytic and damped least-squares IK, manipulability and singularity handling, collision checking. A deliberately unreliable controller drives the arm; the supervisor rejects unsafe commands and measures what that costs.
