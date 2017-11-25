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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/cyrill/ros-playground/src/deedee_driver"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/cyrill/ros-playground/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/cyrill/ros-playground/install/lib/python2.7/dist-packages:/home/cyrill/ros-playground/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/cyrill/ros-playground/build" \
    "/usr/bin/python" \
    "/home/cyrill/ros-playground/src/deedee_driver/setup.py" \
    build --build-base "/home/cyrill/ros-playground/build/deedee_driver" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/cyrill/ros-playground/install" --install-scripts="/home/cyrill/ros-playground/install/bin"
