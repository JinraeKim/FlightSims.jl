abstract type AbstractPNG <: AbstractController end

struct PPNG <: AbstractPNG
    N
end

function Command(guidance::PPNG)
    @unpack N = guidance
    return function (p_M, v_M, p_T, v_T)
        Ω = cross(p_T-p_M, v_T-v_M) / dot(p_T-p_M, p_T-p_M)
        a_M = N * cross(Ω, v_M)
    end
end
