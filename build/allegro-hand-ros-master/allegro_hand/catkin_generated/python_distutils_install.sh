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

echo_and_run cd "/home/joe/sg_ws/src/allegro-hand-ros-master/allegro_hand"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/joe/sg_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/joe/sg_ws/install/lib/python3/dist-packages:/home/joe/sg_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/joe/sg_ws/build" \
    "/usr/bin/python3" \
    "/home/joe/sg_ws/src/allegro-hand-ros-master/allegro_hand/setup.py" \
     \
    build --build-base "/home/joe/sg_ws/build/allegro-hand-ros-master/allegro_hand" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/joe/sg_ws/install" --install-scripts="/home/joe/sg_ws/install/bin"
