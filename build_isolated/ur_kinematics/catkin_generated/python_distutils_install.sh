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

echo_and_run cd "/home/ubuntu/p5_project/src/fmauch_universal_robot/ur_kinematics"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ubuntu/p5_project/install_isolated/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ubuntu/p5_project/install_isolated/lib/python3/dist-packages:/home/ubuntu/p5_project/build_isolated/ur_kinematics/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ubuntu/p5_project/build_isolated/ur_kinematics" \
    "/usr/bin/python3" \
    "/home/ubuntu/p5_project/src/fmauch_universal_robot/ur_kinematics/setup.py" \
     \
    build --build-base "/home/ubuntu/p5_project/build_isolated/ur_kinematics" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ubuntu/p5_project/install_isolated" --install-scripts="/home/ubuntu/p5_project/install_isolated/bin"
