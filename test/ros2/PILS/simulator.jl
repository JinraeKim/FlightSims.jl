using PyCall

rclpy = pyimport("rclpy")
rosNode = pyimport("rclpy.node")
geoMsg = pyimport("geometry_msgs.msg")


@pydef mutable struct StateNode <: rosNode.Node
    function __init__(self)
        rosNode.Node.__init__(self, "state_node")
        # publisher
        self.publisher_ = self.create_publisher(geoMsg.Point, "state", 10)
        timer_period = 0.1
        function timer_callback(self)
            msg_state = geoMsg.Point()
            msg_state.x = self.state[1]
            msg_state.y = self.state[2]
            msg_state.z = self.state[3]
            self.publisher_.publish(msg_state)
            self.get_logger().info("state: $(self.state)")
            if self.control != nothing
                self.state += self.control
            end
        end
        self.timer = self.create_timer(timer_period, () -> timer_callback(self))
        # subscriber
        self.state = ones(3)
        self.control = zeros(3)  # TODO: change
        function listener_callback(self, msg_control)
            self.control = [msg_control.x, msg_control.y, msg_control.z]
            # self.control = np.array(msg_control.data)
            # self.get_logger().info("I heard: $(msg.data)")
        end
        self.subscription = self.create_subscription(geoMsg.Point, "control", msg -> listener_callback(self, msg), 10)
    end
end

function main(args=nothing)
    rclpy.init(args=args)
    state_node = StateNode()
    rclpy.spin(state_node)
    state_node.destroy_node()
    rclpy.shutdown()
end

main()
