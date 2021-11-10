using PyCall

rclpy = pyimport("rclpy")
rosNode = pyimport("rclpy.node")
rosString = pyimport("std_msgs.msg")


@pydef mutable struct MinimialPublisher <: rosNode.Node
    function __init__(self)
        rosNode.Node.__init__(self, "minimal_publisher")
        self.publisher_ = self.create_publisher(rosString.String, "topic", 10)
        timer_period = 0.5
        self.i = 0
        function timer_callback(self)
            msg = rosString.String()
            msg.data = "Hello world: $(self.i)"
            self.publisher_.publish(msg)
            self.get_logger().info("Publishing: $(msg.data)")
            self.i = self.i + 1
        end
        self.timer = self.create_timer(timer_period, () -> timer_callback(self))
    end
end


function main(args=nothing)
    rclpy.init(args=args)
    minimal_publisher = MinimialPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
end
