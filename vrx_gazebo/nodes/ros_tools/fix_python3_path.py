import sys

for path in sys.path:
    print(path)
    
sys.path = [
    path.replace("python2.7", "python3") if "/devel/lib/" in path else path
    for path in sys.path
    if path != "/usr/local/lib/python2.7/dist-packages"
]


"""
Change
/home/argrobotx/robotx-2022/catkin_ws/devel/lib/python2.7/dist-packages
to
/home/argrobotx/robotx-2022/catkin_ws/devel/lib/python3/dist-packages

And remove this line
/usr/local/lib/python2.7/dist-packages
"""
