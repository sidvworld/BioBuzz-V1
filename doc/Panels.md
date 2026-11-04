# Robot pose in Panels

Robot pose is streamed during autonomous and teleop after the drivetrain updates.
Connect to the Robot Controller Wi-Fi and open <http://192.168.43.1:8001>.
For USB access, run `adb forward tcp:8001 tcp:8001` and open <http://localhost:8001>.

Add the Field and Telemetry widgets to see the robot marker, heading line,
X/Y in inches, and heading in degrees. The field uses the Pedro Pathing
coordinate preset; the marker has a 9-inch display radius.

The pose comes from the drivetrain's existing follower/localizer. Starting poses
and autonomous-to-teleop pose transfer continue to be set by the OpModes.

Panels documentation: <https://panels.bylazar.com/docs/com.bylazar.docs/>
