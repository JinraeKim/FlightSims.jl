struct BacksteppingPositionController_StaticAllocator_MulticopterEnv <: AbstractEnv
    controller::BacksteppingPositionControllerEnv
    allocator::StaticAllocator
    multicopter::MulticopterEnv
end

# outer constructor
function BacksteppingPositionController_StaticAllocator_MulticopterEnv(pos_cmd_func=nothing)
    multicopter = LeeHexacopterEnv()
    @unpack m, B = multicopter
    controller = BacksteppingPositionControllerEnv(m; pos_cmd_func=pos_cmd_func)
    allocator = PseudoInverseAllocator(B)
    env = BacksteppingPositionController_StaticAllocator_MulticopterEnv(controller, allocator, multicopter)
end

function State(env::BacksteppingPositionController_StaticAllocator_MulticopterEnv)
    @unpack controller, allocator, multicopter = env
    @unpack m, g = multicopter
    function state(; args_multicopter=())
        x_multicopter = State(multicopter)(args_multicopter...)
        pos0 = copy(x_multicopter.p)
        x_controller = State(controller)(pos0, m, g)
        ComponentArray(multicopter=x_multicopter, controller=x_controller)
    end
end

function Dynamics!(env::BacksteppingPositionController_StaticAllocator_MulticopterEnv)
    @unpack controller, allocator, multicopter = env
    @unpack m, J, g = multicopter
    @Loggable function dynamics!(dx, x, params, t; pos_cmd=nothing)
        @unpack p, v, R, ω = x.multicopter
        @unpack ref_model, Td = x.controller
        xd, vd, ad, ȧd, äd = ref_model.x_0, ref_model.x_1, ref_model.x_2, ref_model.x_3, ref_model.x_4
        νd, Ṫd, _... = Command(controller)(
                                           p, v, R, ω,
                                           xd, vd, ad, ȧd, äd, Td,
                                           m, J, g,
                                          )
        u_cmd = allocator(νd)
        @nested_log :multicopter FlightSims._Dynamics!(multicopter)(dx.multicopter, x.multicopter, (), t; u=u_cmd)
        @nested_log :controller Dynamics!(controller)(dx.controller, x.controller, (), t; pos_cmd=pos_cmd, Ṫd=Ṫd)
        @nested_log :controller νd
    end
end
