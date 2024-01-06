car-only:
    ros2 launch sim_pkg map_with_car.launch

all-objects:
    #! /bin/sh
    #
    # `map_with_all_objects.launch` can sometimes fill the queue and not
    # allow all objects to launch. This is a hack to delay launching the
    # objects hopefully after launching gazebo. The `trap` is a hack to
    # ensure SIGINT propagates to both processes. Obviously, this is non-
    # deterministic. Obviously, I don't care.

    # https://stackoverflow.com/a/2173421
    trap "trap - TERM && kill -- -$$" INT TERM EXIT

    ros2 launch sim_pkg map_with_car.launch &
    sleep 5
    ros2 launch sim_pkg all_objects.launch &
    wait

control_example:
    ros2 run example control

camera_example:
    ros2 run example camera
