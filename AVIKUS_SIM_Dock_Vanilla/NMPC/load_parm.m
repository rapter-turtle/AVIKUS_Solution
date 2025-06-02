function P = load_parm()    

    % Actuator Dynamics
    P.DelRate = 100*3.141592/180; % deg/s
    P.DelMax = 30*3.141592/180;
    P.ThrRate = 100; % percent/s
    P.ThrMax = 100;

    % Geometric
    P.L = 9;
    P.B = 3;
    P.M = 4000; % mass [kg], Maximum mass is 4600
    P.kzz = 0.25*P.L;
    P.Izz = P.kzz^2*P.M;
    P.yp = 0.5;
    P.xp = 2;
    P.b = 15.722;
    P.AT = 4;
    P.AL = 10;

    % % Dynamic model
    % P.Xu    =  -189.76;
    % P.Xvv   =  54.32;
    % P.Xuvv  =  167.74;
    % P.Xvvvv =  2076.9;
    % P.Xrr   =  -15842;
    % P.Xvr   =  -756.37;
    % P.Yv    =  -950.31;
    % P.Yuv   =  818.67;
    % P.Yvv   =  -3054.3;
    % P.Yuvv  =  -1589.1;
    % P.Yr    =  1183.2;
    % P.Yrr   =  9661.3;
    % P.Yvvr  =  54001;
    % P.Yvrr  =  71332;
    % P.Nv    =  540.22;
    % P.Nuv   =  -303.61;
    % P.Nvv   =  1244.9;
    % P.Nuvv  =  188.83;
    % P.Nr    =  -6530.9;
    % P.Nrr   =  23075;
    % P.Nvvr  =  -36088;
    % P.Nvrr  =  20781;
    % P.KPfwd =  1.7;
    % P.KPrvs =  1.0;    

    % Dynamic model
    P.Xu    = -173.135;
    P.Xvv   = -219.9628;
    P.Xuvv  = 511.2493;
    P.Xvvvv = 1995;
    P.Xrr   = -12961;
    P.Xvr   = 2077;
    P.Yv    = -837.5729;
    P.Yuv   = 346.7818;
    P.Yvv   = -324.6011;
    P.Yuvv  = 168.4360;
    P.Yr    = 2865.6/2;
    P.Yrr   = 24330/2;
    P.Yvvr  = 2492.3/100;
    P.Yvrr  = 45501/100;
    P.Nv    = 478.1453;
    P.Nuv   = -143.9891;
    P.Nvv   = 428.2762;
    P.Nuvv  = 250.3992;
    P.Nr    = -5655.1;
    P.Nrr   = 16659;
    P.Nvvr  = -12651/100;
    P.Nvrr  = 30994/100;
    P.KPfwd = 15.8382;
    P.KPrvs = 12.2293;
end