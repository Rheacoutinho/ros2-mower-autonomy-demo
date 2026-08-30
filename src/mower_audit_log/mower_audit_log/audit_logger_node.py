import json
import os
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class AuditLoggerNode(Node):
    """
    Independent audit-trail logger. Watches safety-relevant topics and
    writes structured JSONL entries only on STATE TRANSITIONS, not every
    message, so the trail stays readable and event-focused.
    """

    def __init__(self):
        super().__init__('audit_logger_node')

        log_dir = os.path.expanduser('~/mower_ws/audit_logs')
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        self.log_path = os.path.join(log_dir, f'audit_{ts}.jsonl')

        self.last_safety_status = None
        self.last_stop_active = None
        self.last_bus_status = None

        self.create_subscription(String, 'safety/status', self.safety_status_cb, 10)
        self.create_subscription(Bool, 'safety/stop_active', self.stop_active_cb, 10)
        self.create_subscription(String, 'vehicle_interface/bus_status', self.bus_status_cb, 10)

        self._write_event('AUDIT_LOG_STARTED', 'audit_logger_node',
                           {'log_file': self.log_path})
        self.get_logger().info(f'Audit logger started, writing to {self.log_path}')

    def _write_event(self, event_type, source, details):
        entry = {
            'timestamp_utc': datetime.now(timezone.utc).isoformat(),
            'event_type': event_type,
            'source': source,
            'details': details,
        }
        with open(self.log_path, 'a') as f:
            f.write(json.dumps(entry) + '\n')

    def safety_status_cb(self, msg):
        if msg.data != self.last_safety_status:
            self._write_event('SAFETY_STATUS_CHANGED', 'safety_monitor_node',
                               {'previous': self.last_safety_status, 'new': msg.data})
            self.last_safety_status = msg.data

    def stop_active_cb(self, msg):
        if msg.data != self.last_stop_active:
            event_type = 'SAFE_STOP_ENGAGED' if msg.data else 'SAFE_STOP_CLEARED'
            self._write_event(event_type, 'safety_monitor_node', {'stop_active': msg.data})
            self.last_stop_active = msg.data

    def bus_status_cb(self, msg):
        status_word = msg.data.split('|')[0].strip()
        if status_word != self.last_bus_status:
            self._write_event('CAN_BUS_STATUS_CHANGED', 'vehicle_interface_node',
                               {'previous': self.last_bus_status, 'new': status_word})
            self.last_bus_status = status_word


def main(args=None):
    rclpy.init(args=args)
    node = AuditLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
