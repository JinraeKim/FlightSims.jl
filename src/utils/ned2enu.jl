function ned2enu(p)
    if length(p) == 3
        return [p[2], p[1], -p[3]]
    else
        return error("Invalid position")
    end
end

function enu2ned(p)
    ned2enu(p)  # actually the same
end
