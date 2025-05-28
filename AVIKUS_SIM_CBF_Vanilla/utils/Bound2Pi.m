function radian_new = Bound2Pi(radian)
    if(radian < -pi)
        radian_new = radian + 2 * pi;
    elseif (radian > pi)
        radian_new = radian - 2 * pi;
    else
        radian_new = radian;
    end
end