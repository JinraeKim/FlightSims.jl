using PyCall
using FlightSims
using FSimROS

rclpy = pyimport("rclpy")
rosNode = pyimport("rclpy.node")
fsim_msg = pyimport("fsim_interfaces.msg")


@pydef mutable struct ControlNode <: rosNode.Node
    function __init__(self)
        rosNode.Node.__init__(self, "control_node")
        # publisher
        self.publisher_ = self.create_publisher(fsim_msg.RotorRateHexa, "control", 10)
        timer_period = 0.1
        function timer_callback(self)
            if self.state != nothing
                msg_control = fsim_msg.RotorRateHexa()
		# put in your custom control law
		self.control = zeros(6)
		msg_control.u1 = self.control[1]
		msg_control.u2 = self.control[2]
		msg_control.u3 = self.control[3]
		msg_control.u4 = self.control[4]
		msg_control.u5 = self.control[5]
		msg_control.u6 = self.control[6]
                self.publisher_.publish(msg_control)
            end
	    self.get_logger().info("control: $(self.control)")
        end
        self.timer = self.create_timer(timer_period, () -> timer_callback(self))
	# env
	self.multicopter = LeeHexacopter()
        # subscriber
        self.state = nothing
        self.control = nothing  # TODO: change
        function listener_callback(self, msg_state)
	    self.state = msg_to_state(self.multicopter, msg_state)
        end
        self.subscription = self.create_subscription(fsim_msg.PoseTwist, "state", msg -> listener_callback(self, msg), 10)
    end
end

function main(args=nothing)
    rclpy.init(args=args)
    println("control node started")
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()
end

main()
