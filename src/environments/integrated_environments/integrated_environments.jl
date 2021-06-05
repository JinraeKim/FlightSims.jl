"""
IslamQuadcopterEnv + BacksteppingPositionControllerEnv
"""
function IslamQuadcopter_BacksteppingControllerEnv(; kwargs_multicopter=Dict(), kwargs_controller=Dict(:x_cmd_func => nothing))
    multicopter = IslamQuadcopterEnv(; kwargs_multicopter...)
    @unpack m = multicopter
    controller = BacksteppingPositionControllerEnv(m; kwargs_controller...)
    multicopter, controller
end

function State(multicopter::IslamQuadcopterEnv, controller::BacksteppingPositionControllerEnv)
    @unpack m, g = multicopter
    return function (args_multicopter=())
        x_multicopter = State(multicopter)(args_multicopter...)
        pos0 = copy(x_multicopter.p)
        x_controller = State(controller)(pos0, m, g)
        ComponentArray(multicopter=x_multicopter, controller=x_controller)
    end
end

function dynamics!(multicopter::IslamQuadcopterEnv, controller::BacksteppingPositionControllerEnv;
    mixer=Mixer(multicopter.B))
    @unpack m, J, g = multicopter
    return function (dx, x, p, t; pos_cmd=nothing)
        @unpack p, v, R, ω = x.multicopter
        @unpack ref_model, Td = x.controller
        xd, vd, ad, ȧd, äd = ref_model.x_0, ref_model.x_1, ref_model.x_2, ref_model.x_3, ref_model.x_4
        νd, Ṫd = command(controller)(p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g)
        u_cmd = mixer(νd)
        dynamics!(multicopter)(dx.multicopter, x.multicopter, (), t; u=u_cmd)
        dynamics!(controller)(dx.controller, x.controller, (), t; pos_cmd=pos_cmd, Ṫd=Ṫd)
        nothing
    end
end

"""
GoodarziQuadcopterEnv + BacksteppingPositionControllerEnv
"""
function GoodarziQuadcopter_BacksteppingControllerEnv(; kwargs_multicopter=Dict(), kwargs_controller=Dict(:x_cmd_func => nothing))
    multicopter = GoodarziQuadcopterEnv(; kwargs_multicopter...)
    @unpack m = multicopter
    controller = BacksteppingPositionControllerEnv(m; kwargs_controller...)
    multicopter, controller
end

function State(multicopter::GoodarziQuadcopterEnv, controller::BacksteppingPositionControllerEnv)
    @unpack m, g = multicopter
    return function (args_multicopter=())
        x_multicopter = State(multicopter)(args_multicopter...)
        pos0 = copy(x_multicopter.p)
        x_controller = State(controller)(pos0, m, g)
        ComponentArray(multicopter=x_multicopter, controller=x_controller)
    end
end

function dynamics!(multicopter::GoodarziQuadcopterEnv, controller::BacksteppingPositionControllerEnv)
    @unpack m, J, g = multicopter
    return function (dx, x, p, t; pos_cmd=nothing)
        @unpack p, v, R, ω = x.multicopter
        @unpack ref_model, Td = x.controller
        xd, vd, ad, ȧd, äd = ref_model.x_0, ref_model.x_1, ref_model.x_2, ref_model.x_3, ref_model.x_4
        νd, Ṫd = command(controller)(p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g)
        dynamics!(controller)(dx.controller, x.controller, (), t; pos_cmd=pos_cmd, Ṫd=Ṫd)
        dynamics!(multicopter)(dx.multicopter, x.multicopter, (), t; f=νd.f, M=νd.M)
        nothing
    end
end
