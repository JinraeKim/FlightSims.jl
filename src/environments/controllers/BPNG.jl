abstract type AbstractBPNG <: AbstractController end

struct BPNG <: AbstractBPNG
    N
    alpha
    delta
    gamma_f_d
    sigma_M_lim
    A_M_max
    n
end

function nzero_sign(x)
    if x >= 0
        y = 1;
    else
        y = -1;
    end
    return y
end

function Command(guidance::BPNG)
    @unpack N, alpha, delta, gamma_f_d, sigma_M_lim, A_M_max, n = guidance
    return function (p_M, v_M, p_T, v_T)
        r       = norm(p_T-p_M)
        lambda  = atan(p_T[2]-p_M[2], p_T[1]-p_M[1])   
        
        V_M     = norm(v_M)
        gamma_M = atan(v_M[2], v_M[1])
        sigma_M = gamma_M - lambda
        eta     = sin(sigma_M)
        eta_lim = sin(sigma_M_lim)

        V_T     = norm(v_T)
        gamma_T = atan(v_T[2], v_T[1])
        sigma_T = gamma_T - lambda

        lambda_dot = (V_T*sin(sigma_T) - V_M*sin(sigma_M)) / r        
        e_gamma_f   = gamma_M - N / (N - 1) * sigma_M - gamma_f_d;
        if alpha > 0.99
            e_gamma_f_fbk = e_gamma_f
        else
            e_gamma_f_fbk = nzero_sign(e_gamma_f)*abs(e_gamma_f)^alpha
        end

        K_r     = (N - 1 + delta) * (1 + r / 10E3)^n ;
        K_eta   = 1 + 9 * (1 - (abs(eta / eta_lim))^10);
        u_aug = - K_r * K_eta / r * e_gamma_f_fbk
        A_M = N * V_M * lambda_dot - u_aug * V_M^2 * cos(sigma_M)
        A_M = min(max(A_M, -A_M_max), A_M_max)
        a_M = A_M * [-sin(gamma_M); cos(gamma_M)]
    end
end

