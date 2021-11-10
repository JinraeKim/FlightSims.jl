using PyCall

rclpy = pyimport("rclpy")
rosNode = pyimport("rclpy.node")
rosString = pyimport("std_msgs.msg")


@pydef mutable struct MinimialSubscriber <: rosNode.Node
    function __init__(self)
        rosNode.Node.__init__(self, "minimal_subscriber")
        function listener_callback(self, msg)
            self.get_logger().info("I heard: $(msg.data)")
        end
        self.subscription = self.create_subscription(rosString.String, "topic", msg -> listener_callback(self, msg), 10)
        self.subscription
    end
end


function main(args=nothing)
    rclpy.init(args=args)
    minimal_subscriber = MinimialSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
end
