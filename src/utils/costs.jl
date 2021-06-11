function QuadraticCost(Q, R)
    return function (x, u)
        x'*Q*x + u'*R*u
    end
end
