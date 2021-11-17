#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/kist/KIST-Dual-Arm-ROS/src/joystick_drivers/wiimote"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/kist/KIST-Dual-Arm-ROS/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/kist/KIST-Dual-Arm-ROS/install/lib/python2.7/dist-packages:/home/kist/KIST-Dual-Arm-ROS/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/kist/KIST-Dual-Arm-ROS/build" \
    "/usr/bin/python2" \
    "/home/kist/KIST-Dual-Arm-ROS/src/joystick_drivers/wiimote/setup.py" \
     \
    build --build-base "/home/kist/KIST-Dual-Arm-ROS/build/joystick_drivers/wiimote" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/kist/KIST-Dual-Arm-ROS/install" --install-scripts="/home/kist/KIST-Dual-Arm-ROS/install/bin"
