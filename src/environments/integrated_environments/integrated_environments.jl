"""
MulticopterEnv + BacksteppingPositionControllerEnv
"""
function State(multicopter::MulticopterEnv, controller::BacksteppingPositionControllerEnv)
    @unpack m, g = multicopter
    return function (args_multicopter=())
        x_multicopter = State(multicopter)(args_multicopter...)
        pos0 = copy(x_multicopter.p)
        x_controller = State(controller)(pos0, m, g)
        ComponentArray(multicopter=x_multicopter, controller=x_controller)
    end
end

function command(controller::BacksteppingPositionControllerEnv, allocator::PseudoInverseControlAllocator,
        p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g
    )
    νd, Ṫd = command(controller)(p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g)
    u_cmd = allocator(νd)
    ComponentArray(νd=νd, Ṫd=Ṫd, u_cmd=u_cmd)
end

function Dynamics!(multicopter::MulticopterEnv,
        controller::BacksteppingPositionControllerEnv,
        allocator::PseudoInverseControlAllocator)
    @unpack m, J, g = multicopter
    return function (dx, x, p, t; pos_cmd=nothing)
        @unpack p, v, R, ω = x.multicopter
        @unpack ref_model, Td = x.controller
        xd, vd, ad, ȧd, äd = ref_model.x_0, ref_model.x_1, ref_model.x_2, ref_model.x_3, ref_model.x_4
        command_info = command(controller, allocator, p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g)
        @unpack νd, Ṫd, u_cmd = command_info
        Dynamics!(multicopter)(dx.multicopter, x.multicopter, (), t; u=u_cmd)
        Dynamics!(controller)(dx.controller, x.controller, (), t; pos_cmd=pos_cmd, Ṫd=Ṫd)
        nothing
    end
end

function Process(multicopter::MulticopterEnv, controller::BacksteppingPositionControllerEnv,
        allocator::PseudoInverseControlAllocator)
    @unpack m, J, g = multicopter
    return function (prob::ODEProblem, sol::ODESolution; Δt=0.01)
        t0, tf = prob.tspan
        ts = t0:Δt:tf
        Xs = ts |> Map(t -> sol(t)) |> collect
        ps = Xs |> Map(x -> x.multicopter.p) |> collect
        vs = Xs |> Map(x -> x.multicopter.v) |> collect
        Rs = Xs |> Map(x -> x.multicopter.R) |> collect
        ωs = Xs |> Map(x -> x.multicopter.ω) |> collect
        xds = Xs |> Map(x -> x.controller.ref_model.x_0) |> collect
        vds = Xs |> Map(x -> x.controller.ref_model.x_1) |> collect
        ads = Xs |> Map(x -> x.controller.ref_model.x_2) |> collect
        ȧds = Xs |> Map(x -> x.controller.ref_model.x_3) |> collect
        äds = Xs |> Map(x -> x.controller.ref_model.x_4) |> collect
        Tds = Xs |> Map(x -> x.controller.Td) |> collect
        u_cmds = (
                  zip(ps, vs, Rs, ωs, xds, vds, ads, ȧds, äds, Tds) |>
                  Map(args -> command(controller, allocator, args..., m, J, g)) |>
                  Map(command_info -> command_info.u_cmd) |>
                  collect
                 )
        DataFrame(
                  time=ts,
                  state=Xs,
                  position=ps,
                  velocity=vs,
                  rotation_matrix=Rs,
                  angular_rate=ωs,
                  u_command=u_cmds,
                 )
    end
end

"""
LeeHexacopterEnv + BacksteppingPositionControllerEnv
"""
function LeeHexacopter_BacksteppingPositionControllerEnv(; kwargs_multicopter=Dict(), kwargs_controller=Dict(:x_cmd_func => nothing))
    multicopter = LeeHexacopterEnv(; kwargs_multicopter...)
    @unpack m = multicopter
    controller = BacksteppingPositionControllerEnv(m; kwargs_controller...)
    multicopter, controller
end

"""
IslamQuadcopterEnv + BacksteppingPositionControllerEnv
"""
function IslamQuadcopter_BacksteppingPositionControllerEnv(; kwargs_multicopter=Dict(), kwargs_controller=Dict(:x_cmd_func => nothing))
    multicopter = IslamQuadcopterEnv(; kwargs_multicopter...)
    @unpack m = multicopter
    controller = BacksteppingPositionControllerEnv(m; kwargs_controller...)
    multicopter, controller
end

"""
GoodarziQuadcopterEnv + BacksteppingPositionControllerEnv
"""
function GoodarziQuadcopter_BacksteppingPositionControllerEnv(; kwargs_multicopter=Dict(), kwargs_controller=Dict(:x_cmd_func => nothing))
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

function Dynamics!(multicopter::GoodarziQuadcopterEnv, controller::BacksteppingPositionControllerEnv)
    @unpack m, J, g = multicopter
    return function (dx, x, p, t; pos_cmd=nothing)
        @unpack p, v, R, ω = x.multicopter
        @unpack ref_model, Td = x.controller
        xd, vd, ad, ȧd, äd = ref_model.x_0, ref_model.x_1, ref_model.x_2, ref_model.x_3, ref_model.x_4
        νd, Ṫd = command(controller)(p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g)
        Dynamics!(controller)(dx.controller, x.controller, (), t; pos_cmd=pos_cmd, Ṫd=Ṫd)
        Dynamics!(multicopter)(dx.multicopter, x.multicopter, (), t; f=νd.f, M=νd.M)
        nothing
    end
end
