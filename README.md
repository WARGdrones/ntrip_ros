# ntrip_ros
NTRIP client, imports RTCM streams to ROS topic

This was forked from github.com/tilk/ntrip_ros

The CORS correction server that I am using does not have the /n/r characters. So I parsed out individual messages and published each one on the /rtcm ROS topic.
It would crash with IncompleteRead error. I added patch at top of file.
But the connection had closed and it would crash again. I ended up detecting zero length data and closing and reopening the data stream.
It continues on without a glitch.

You can generate the require $GPGGA message at this site. https://www.nmeagen.org/ Set a point near where you want to run and click "Generate NMEA file". Cut and paste the $GPGGA message into the launch file.

I intend to use it with https://github.com/ros-agriculture/ublox_f9p

It may also require this package: https://github.com/tilk/rtcm_msgs

A similar NTRIP client (may be better than mine) is here: https://github.com/dayjaby/ntrip_ros

# updated by WARGdrones by merging all relevant forks together
- based on https://github.com/ros-agriculture/ntrip_ros with better error handling
- merged https://github.com/dayjaby/ntrip_ros to only use mavros builtin messages
- python3 changes from https://github.com/Autonabit/ntrip_ros
- calculating GGA message instead of using a fixed one: https://github.com/duwke/ntrip_ros/
- Also fixing the GGA-String generation (the checksum calculation was wrong)
- (The node will expect all the necessary data to be in env variables or provided in hub_state, we are running it inside of docker)
- Expanded by starting and stopping from inside ROS
