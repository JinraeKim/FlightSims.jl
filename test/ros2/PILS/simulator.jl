using PyCall
using FlightSims
using FSimROS
using Plots
using FSimPlots

rclpy = pyimport("rclpy")
rosNode = pyimport("rclpy.node")
fsim_msg = pyimport("fsim_interfaces.msg")


struct Multicopter_ZOH_Input <: AbstractEnv
    multicopter::Multicopter
end

function FSimBase.State(env::Multicopter_ZOH_Input)
    State(env.multicopter)
end

function FSimBase.Dynamics!(env::Multicopter_ZOH_Input)
    @Loggable function dynamics!(dx, x, input, t)
	@nested_log FSimZoo._Dynamics!(env.multicopter)(dx, x, nothing, t; u=input)
    end
end


@pydef mutable struct StateNode <: rosNode.Node
    function __init__(self)
        rosNode.Node.__init__(self, "state_node")
        # publisher
        self.publisher_ = self.create_publisher(fsim_msg.PoseTwist, "state", 10)
        timer_period = 0.1
	self.fig = plot()
        function timer_callback(self)
            # msg_state = geoMsg.Point()
            # msg_state.x = self.state[1]
            # msg_state.y = self.state[2]
            # msg_state.z = self.state[3]
	    msg = state_to_msg(self.env.multicopter, copy(self.simulator.integrator.u))
            self.publisher_.publish(msg)
            self.get_logger().info("state: $(self.simulator.integrator.u)")
	    if self.control_received
	        step_until!(self.simulator, self.simulator.integrator.t + timer_period)
	    end
	    plot!(self.fig, self.env.multicopter, copy(self.simulator.integrator.u); xlim=(-10, 10), ylim=(-10, 10), zlim=(-10, 10))
	    display(self.fig)
	    self.fig = plot()
        end
        self.timer = self.create_timer(timer_period, () -> timer_callback(self))
	# env
	multicopter = LeeHexacopter()
	self.env = Multicopter_ZOH_Input(multicopter)
	state0 = State(self.env)()
	u0 = zeros(6)  # initial control input
	self.simulator = Simulator(state0, Dynamics!(self.env), u0; tf=10_000)  # second
        # subscriber
        self.control_received = false
	# self.control = zeros(6)  # TODO: change
        function listener_callback(self, msg_control)
		@show self.control_received
		if !self.control_received
			self.control_received = true
		end
	    input = [msg_control.u1, msg_control.u2, msg_control.u3, msg_control.u4, msg_control.u5, msg_control.u6]
	    self.simulator.integrator.p = input  # TODO
        end
        self.subscription = self.create_subscription(fsim_msg.RotorRateHexa, "control", msg -> listener_callback(self, msg), 10)
    end
end

function main(args=nothing)
    rclpy.init(args=args)
    println("state node started")
    state_node = StateNode()
    rclpy.spin(state_node)
    state_node.destroy_node()
    rclpy.shutdown()
end

main()
