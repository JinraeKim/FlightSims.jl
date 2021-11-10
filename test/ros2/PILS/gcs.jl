using PyCall

rclpy = pyimport("rclpy")
rosNode = pyimport("rclpy.node")
geoMsg = pyimport("geometry_msgs.msg")


@pydef mutable struct ControlNode <: rosNode.Node
    function __init__(self)
        rosNode.Node.__init__(self, "control_node")
        # publisher
        self.publisher_ = self.create_publisher(geoMsg.Point, "control", 10)
        timer_period = 0.5
        function timer_callback(self)
            if self.state != nothing
                msg_control = geoMsg.Point()
                self.control = -0.1*self.state
                msg_control.x = self.control[1]
                msg_control.y = self.control[2]
                msg_control.z = self.control[3]
                self.publisher_.publish(msg_control)
                self.get_logger().info("control: $(self.control)")
            end
        end
        self.timer = self.create_timer(timer_period, () -> timer_callback(self))
        # subscriber
        self.state = nothing
        self.control = zeros(3)  # TODO: change
        function listener_callback(self, msg_state)
            self.state = [msg_state.x, msg_state.y, msg_state.z]
        end
        self.subscription = self.create_subscription(geoMsg.Point, "state", msg -> listener_callback(self, msg), 10)
    end
end

function main(args=nothing)
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()
end

main()
