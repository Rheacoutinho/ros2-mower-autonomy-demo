import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class SimulatedCanFrame:
    """
    Stand-in for a real CAN frame. NOT a real bit-packed CAN payload --
    just a structured representation of what one would carry, so the
    translation logic and interface contract are explicit and explainable.

    A real frame for a drive-by-wire command would typically be:
      - arbitration_id: fixed ID identifying this message type on the bus
      - 8 data bytes, individual signals packed into specific bit ranges
        per a DBC (CAN database) file, with defined scaling/offset per
        signal (e.g. velocity might be an int16 scaled by 0.001 m/s/bit)
      - a rolling counter and/or checksum byte for message integrity
    """

    def __init__(self, arbitration_id, signal_name, value, unit):
        self.arbitration_id = arbitration_id
        self.signal_name = signal_name
        self.value = value
        self.unit = unit
        self.timestamp = time.time()

    def __repr__(self):
        return (f'CAN[id=0x{self.arbitration_id:03X}] '
                f'{self.signal_name}={self.value:.3f}{self.unit}')


class VehicleInterfaceNode(Node):
    """
    Simulated CAN / drive-by-wire abstraction layer.

    HONEST LIMITATION (flag explicitly): there is no real CAN bus, no real
    DBC file, no real transceiver hardware here. This node demonstrates the
    ARCHITECTURE and INTERFACE CONTRACT a real vehicle interface layer would
    have -- command translation into bus-style messages, a heartbeat/timeout
    safety check, and telemetry feedback -- using Python objects shaped like
    CAN signals rather than real bit-packed frames on a real bus. In a real
    system this node would be replaced by a driver using a library like
    python-can (or a compiled driver) talking to real transceiver hardware,
    with signal encoding defined by an actual DBC file matching the vehicle
    ECU's specification.

    What IS realistic here: the conceptual separation of "ROS2-side command"
    from "bus-side signal," the presence of a command timeout/heartbeat
    check (a real safety-relevant CAN behaviour -- if commands stop
    arriving, a real drive-by-wire ECU should fault-stop, not keep
    executing a stale command), and the read-back telemetry loop.
    """

    # Fake arbitration IDs, styled like a real interface spec would define
    CAN_ID_DRIVE_CMD = 0x201
    CAN_ID_STEER_CMD = 0x202
    CAN_ID_VEHICLE_STATUS = 0x301

    def __init__(self):
        super().__init__('vehicle_interface_node')

        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.cmd_timeout_sec = self.get_parameter('cmd_timeout_sec').value

        self.last_cmd_time = None
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.status_pub = self.create_publisher(String, 'vehicle_interface/bus_status', 10)
        self.out_pub = self.create_publisher(Twist, 'vehicle_interface/actuated_cmd', 10)

        # 20 Hz "bus" tick -- simulates the interface layer running its own
        # loop independent of when ROS2 messages arrive, the way a real
        # embedded interface node polling/writing a CAN bus would.
        self.timer = self.create_timer(0.05, self.bus_tick)

        self.get_logger().info(
            f'Vehicle interface node started (simulated CAN), '
            f'timeout={self.cmd_timeout_sec}s')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.time()
        self.last_linear_x = msg.linear.x
        self.last_angular_z = msg.angular.z

    def bus_tick(self):
        now = time.time()

        # Heartbeat / command-timeout check -- a real safety-relevant
        # behaviour of drive-by-wire interfaces: if the upstream commander
        # goes silent (crash, network partition, hang), the interface layer
        # must NOT keep replaying the last command indefinitely.
        cmd_stale = (
            self.last_cmd_time is None
            or (now - self.last_cmd_time) > self.cmd_timeout_sec
        )

        if cmd_stale:
            linear_x = 0.0
            angular_z = 0.0
            bus_status = 'FAULT_CMD_TIMEOUT' if self.last_cmd_time is not None else 'NO_CMD_YET'
        else:
            linear_x = self.last_linear_x
            angular_z = self.last_angular_z
            bus_status = 'OK'

        # Build the simulated CAN frames -- this is the "translation" step
        drive_frame = SimulatedCanFrame(
            self.CAN_ID_DRIVE_CMD, 'DriveVelocity', linear_x, 'm/s')
        steer_frame = SimulatedCanFrame(
            self.CAN_ID_STEER_CMD, 'SteerRate', angular_z, 'rad/s')

        # In a real driver: encode these into actual bytes and write to the
        # bus here (e.g. bus.send(can.Message(...))). Here we just log the
        # structured representation, so the translation step is visible and
        # explainable.
        if bus_status != 'OK':
            self.get_logger().warn(
                f'{bus_status}: zeroing actuation. Last frame would be '
                f'{drive_frame}, {steer_frame}')

        # Publish what "actually reached the actuator" after this layer --
        # this is the signal the vehicle model should really listen to in
        # a fully wired system, since it reflects timeout-safety too.
        out_msg = Twist()
        out_msg.linear.x = linear_x
        out_msg.angular.z = angular_z
        self.out_pub.publish(out_msg)

        status_msg = String()
        status_msg.data = (
            f'{bus_status} | {drive_frame} | {steer_frame}'
        )
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
