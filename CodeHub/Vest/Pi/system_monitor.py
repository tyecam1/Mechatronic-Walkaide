#!/usr/bin/env python3
import rospy
import psutil
import json
from diagnostic_updater import Updater, DiagnosticStatusWrapper
from std_msgs.msg import String
import socket

rospy.init_node("system_monitor")
machine_name = socket.gethostname()

sys_pub = rospy.Publisher("/system_diagnostics", String, queue_size=10)

updater = Updater()
updater.setHardwareID(machine_name)

def diagnostic_callback(stat):
    cpu_usage = psutil.cpu_percent(interval=0.5)
    memory_info = psutil.virtual_memory()
    stat.summary(DiagnosticStatusWrapper.OK, "Monitoring CPU and memory")
    stat.add("CPU Usage (%)", cpu_usage)
    stat.add("Memory Usage (%)", memory_info.percent)
    sys_pub.publish(json.dumps({"machine": machine_name, "cpu": cpu_usage, "memory": memory_info.percent}))
    return stat

updater.add(f"System Load ({machine_name})", diagnostic_callback)

rospy.loginfo(f"System Monitor started on {machine_name}")
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    updater.update()
    rate.sleep()

