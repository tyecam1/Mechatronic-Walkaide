#!/usr/bin/env python3
import rospy
import psutil
import json
from diagnostic_updater import Updater, DiagnosticStatusWrapper
from std_msgs.msg import String

rospy.init_node("system_monitor")

# Get hostname to distinguish between Pi5 and Panda
import socket
machine_name = socket.gethostname()

# Publisher for system diagnostics
sys_pub = rospy.Publisher("/system_diagnostics", String, queue_size=10)

# Diagnostic updater
updater = Updater()
updater.setHardwareID(machine_name)

def diagnostic_callback(stat):
    """Publishes CPU and memory usage diagnostics."""
    cpu_usage = psutil.cpu_percent(interval=0.5)
    memory_info = psutil.virtual_memory()
    
    diag = DiagnosticStatusWrapper()
    diag.name = f"System Diagnostics ({machine_name})"
    diag.level = DiagnosticStatusWrapper.OK
    diag.message = "Monitoring CPU and memory"
    diag.add("CPU Usage (%)", cpu_usage)
    diag.add("Memory Usage (%)", memory_info.percent)

    # Publish JSON message for logging
    sys_pub.publish(json.dumps({"machine": machine_name, "cpu": cpu_usage, "memory": memory_info.percent}))
    
    stat.add(diag)

updater.add(f"System Load ({machine_name})", diagnostic_callback)

rospy.loginfo(f"System Monitor started on {machine_name}")
rate = rospy.Rate(1)  # Update every second
while not rospy.is_shutdown():
    updater.update()
    rate.sleep()
