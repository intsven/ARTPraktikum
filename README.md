Für Pfadverfolgung mit Odom
1. "catkin_make" im Catkin Workspace

2. "roslaunch volksbot messtechnikpraktikum.launch"

3. Zu verfolgenden Pfad in "path.dat" umbenennen.

4. "rosrun aufgaben robot_control" in dem Verzeichnis mit "path.dat"

Für Pfadverfolgung mit AMCL
1. "catkin_make" im Catkin Workspace

2.1 "roslaunch volksbot messtechnikpraktikum.launch"
2.2 "roslaunch volksbot messtechnikamcl.launch"

3. Zu verfolgenden Pfad in "path.dat" umbenennen.

4. "rosrun aufgaben robot_control_amcl" in dem Verzeichnis mit "path.dat"