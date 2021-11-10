using PyCall

rclpy = pyimport("rclpy")
rosNode = pyimport("rclpy.node")
FSim_msg = pyimport("fsim_interfaces.msg")
np = pyimport("numpy")


@pydef mutable struct StateNode <: rosNode.Node
    function __init__(self, state0)
        rosNode.Node.__init__(self, "state_node")
        # publisher
        self.publisher_ = self.create_publisher(FSim_msg.Data, "state", 10)
        timer_period = 0.5
        function timer_callback(self)
            msg_state = FSim_msg.Data()
            msg_state.data = self.state  # TODO: check it
            # msg_state.data = self.state.tolist()
            self.publisher_.publish(msg_state)
            self.get_logger().info("state: $(msg_state.data)")
            if self.control != nothing
                self.state += -0.1*self.state
            end
        end
        self.timer = self.create_timer(timer_period, () -> timer_callback(self))
        # subscriber
        self.state = state0
        self.control = zeros(3)  # TODO: change
        function listener_callback(self, msg_control)
            # self.control = np.array(msg_control.data)
            # self.get_logger().info("I heard: $(msg.data)")
        end
        self.subscription = self.create_subscription(FSim_msg.Data, "control", msg -> listener_callback(self, msg), 10)
    end
end

function main(args=nothing)
    rclpy.init(args=args)
    state_node = StateNode()
    rclpy.spin(state_node)
    state_node.destroy_node()
    rclpy.shutdown()
end
