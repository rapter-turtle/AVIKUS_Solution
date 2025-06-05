function [TP_cmd, TS_cmd, delPR_cmd, delSR_cmd] = thrust_cbf(TP_cmd0, TS_cmd0, delPR_cmd0, delSR_cmd0, u_state, v_state, r_state, x_state, y_state, psi_state, alpha1, alpha2, alpha1_h, alpha2_h, alpha_l, alpha_r, a, control_update_dt)
        P = load_parm();

        Xu    =  -189.76;
        Yv    =  -950.31;
        Yr    =  1183.2;
        Nv    =  540.22;
        Nr    =  -6530.9;
        
        TP_x0 = TP_cmd0*cos(delPR_cmd0);
        TP_y0 = -TP_cmd0*sin(delPR_cmd0);
        TS_x0 = TS_cmd0*cos(delSR_cmd0);
        TS_y0 = -TS_cmd0*sin(delSR_cmd0);

        TP_x = sdpvar(1,1);      
        TP_y = sdpvar(1,1);      
        TS_x = sdpvar(1,1);      
        TS_y = sdpvar(1,1);      
        s1 = sdpvar(1,1);
        s2 = sdpvar(1,1);
        s3 = sdpvar(1,1);

        %% Define Cost Function
        Objective = (TP_x - TP_x0)^2 + (TP_y - TP_y0)^2 + (TS_x - TS_x0)^2 + (TS_y - TS_y0)^2 + 1e9 * s1^2 + 1e9 * s2^2 + 1e9 * s3^2;

        %% Define Constraints (CBF)
        Constraints = [];
       
        %% Preprocess
        % uvr dot
        Tu = (TP_x + TS_x)/P.M;
        Tv = (TP_y + TS_y)/P.M;
        Tr = (- P.xp*(TP_y + TS_y) + P.yp*(TP_x - TS_x))/P.Izz;

        u_dot = (P.Xu*u_state)/P.M;
        v_dot = (P.Yv*v_state + P.Yr*r_state)/P.M;
        r_dot = (P.Nv*v_state + P.Nr*r_state)/P.Izz;
       

        %% heading CBF
        heading = -0.5*pi;
        heading_bound = 30*pi/180;
        heading_cbf = -(psi_state - heading)^2 + heading_bound^2;
        heading_cbf_dot = -2*r_state*(psi_state - heading);

        heading_cbf_dotdot = -2*r_state*r_state - 2*(psi_state - heading)*(r_dot + Tr);
        
        head_cbf_fea_control = 2*P.xp*P.Fy_max + 2*P.yp*P.Fx_max;
        heading_cbf_fea = (alpha1_h + alpha2_h)*heading_cbf_dot + alpha1_h*alpha2_h*heading_cbf -2*r_state*r_state - 2*(psi_state - heading)*(Nv*v_state + Nr*r_state + head_cbf_fea_control)/P.Izz;
        heading_cbf_fea_dot = (alpha1_h + alpha2_h)*heading_cbf_dotdot + alpha1_h*alpha2_h*heading_cbf_dot -4*r_state*(Nv*v_state + Nr*r_state + Tr)/P.Izz - 2*r_state*(Nv*v_state + Nr*r_state + head_cbf_fea_control)/P.Izz - 2*(psi_state - heading)*(Nv*(v_dot + Tv) + Nr*(r_dot + Tr))/P.Izz;

        %% Left_Right CBF
        xdot = u_state*cos(psi_state) - v_state*sin(psi_state);
        ydot = u_state*sin(psi_state) + v_state*cos(psi_state);
        
        u_dotdot = (P.Xu*(u_dot + Tu))/P.M;
        v_dotdot = (P.Yv*(v_dot + Tv) + P.Yr*(r_dot + Tr))/P.M;

        xdotdot = u_dot*cos(psi_state) - v_dot*sin(psi_state) - u_state*r_state*sin(psi_state) - v_state*r_state*cos(psi_state);
        ydotdot = u_dot*sin(psi_state) + v_dot*cos(psi_state) + u_state*r_state*cos(psi_state) - v_state*r_state*sin(psi_state);
        
        xdotdotdot = u_dotdot*cos(psi_state) - (u_dot + Tu)*r_state*sin(psi_state) ...
                    - v_dotdot*sin(psi_state) - (v_dot + Tv)*r_state*cos(psi_state) ...
                    - (u_dot + Tu)*r_state*sin(psi_state) - u_state*(r_dot + Tr)*sin(psi_state) - u_state*r_state*r_state*cos(psi_state) ...
                    - (v_dot + Tv)*r_state*cos(psi_state) - v_state*(r_dot + Tr)*cos(psi_state) + v_state*r_state*r_state*cos(psi_state);

        ydotdotdot = u_dotdot*sin(psi_state) + (u_dot + Tu)*r_state*cos(psi_state) ...
                    + v_dotdot*cos(psi_state) - (v_dot + Tv)*r_state*sin(psi_state) ...
                    + (u_dot + Tu)*r_state*cos(psi_state) + u_state*(r_dot + Tr)*cos(psi_state) - u_state*r_state*r_state*sin(psi_state) ...
                    - (v_dot + Tv)*r_state*sin(psi_state) - v_state*(r_dot + Tr)*sin(psi_state) - v_state*r_state*r_state*cos(psi_state);
        
        %% Left CBF
        left_cbf = -y_state + a*x_state;
        left_cbf_dot = -ydot + a*xdot;
        left_cbf_dotdot = -(ydotdot + Tu*sin(psi_state) + Tv*cos(psi_state)) + a*(xdotdot + Tu*cos(psi_state) - Tv*sin(psi_state));
        
        left_cbf_fea_control = abs((a*cos(psi_state) - sin(psi_state))*2*P.Fx_max/P.M) + abs((cos(psi_state) + a*sin(psi_state))*2*P.Fy_max/P.M);
        
        left_cbf_fea = (alpha1 + alpha2)*left_cbf_dot + alpha1*alpha2*left_cbf - ydotdot + a*xdotdot + left_cbf_fea_control;
        left_cbf_fea_dot = (alpha1 + alpha2)*left_cbf_dotdot + alpha1*alpha2*left_cbf_dot - ydotdotdot + a*xdotdotdot;

        %% Left CBF
        b = -a;
        right_cbf = -y_state + b*x_state;
        right_cbf_dot = -ydot + b*xdot;
        right_cbf_dotdot = -(ydotdot + Tu*sin(psi_state) + Tv*cos(psi_state)) + b*(xdotdot + Tu*cos(psi_state) - Tv*sin(psi_state));
        
        right_cbf_fea_control = abs((b*cos(psi_state) - sin(psi_state))*2*P.Fx_max/P.M) + abs((cos(psi_state) + b*sin(psi_state))*2*P.Fy_max/P.M);
        
        right_cbf_fea = (alpha1 + alpha2)*right_cbf_dot + alpha1*alpha2*right_cbf - ydotdot + b*xdotdot + right_cbf_fea_control;
        right_cbf_fea_dot = (alpha1 + alpha2)*right_cbf_dotdot + alpha1*alpha2*right_cbf_dot - ydotdotdot + b*xdotdotdot;


        %% Nominal
        % heading CBF
        Constraints = [Constraints, (alpha1 + alpha2)*heading_cbf_dot + alpha1*alpha2*heading_cbf + heading_cbf_dotdot + s1 >= 0];

        % Left CBF
        Constraints = [Constraints, (alpha1 + alpha2)*left_cbf_dot + alpha1*alpha2*left_cbf + left_cbf_dotdot + s2 >= 0];

        % Right CBF
        Constraints = [Constraints, (alpha1 + alpha2)*right_cbf_dot + alpha1*alpha2*right_cbf + right_cbf_dotdot + s3 >= 0];



        %% fea_Nominal
        % % % heading CBF
        % Constraints = [Constraints, heading_cbf_fea_dot + alpha_l*heading_cbf_fea + s1 >= 0];
        % % 
        % % % Left CBF
        % Constraints = [Constraints, left_cbf_fea_dot + alpha_l*left_cbf_fea + s2 >= 0];
        % % 
        % % % Right CBF
        % Constraints = [Constraints, right_cbf_fea_dot + alpha_r*right_cbf_fea + s3 >= 0];


        %% Define Constraints (ux, uy)
        % Constraints = [Constraints, TP_x <= TP_x0 + 0.5*P.Fx_max];
        % Constraints = [Constraints, TP_x >= TP_x0 - 0.5*P.Fx_max];
        % Constraints = [Constraints, TP_y <= TP_y0 + 0.5*P.Fy_max];
        % Constraints = [Constraints, TP_y >= TP_y0 - 0.5*P.Fy_max];
        % 
        % Constraints = [Constraints, TS_x <= TS_x0 + 0.1*P.Fx_max];
        % Constraints = [Constraints, TS_x >= TS_x0 - 0.1*P.Fx_max];
        % Constraints = [Constraints, TS_y <= TS_y0 + 0.1*P.Fy_max];
        % Constraints = [Constraints, TS_y >= TS_y0 - 0.1*P.Fy_max];

        Constraints = [Constraints, TP_x <= P.Fx_max];
        Constraints = [Constraints, TP_x >= -P.Fx_max];
        Constraints = [Constraints, TP_y <= P.Fy_max];
        Constraints = [Constraints, TP_y >= -P.Fy_max];

        Constraints = [Constraints, TS_x <= P.Fx_max];
        Constraints = [Constraints, TS_x >= -P.Fx_max];
        Constraints = [Constraints, TS_y <= P.Fy_max];
        Constraints = [Constraints, TS_y >= -P.Fy_max];

        %% Solve using SDPT3
        options = sdpsettings('solver', 'sdpt3', 'verbose', 0);
        sol = optimize(Constraints, Objective, options);

        TP_x_sol = value(TP_x);
        TS_x_sol = value(TS_x);
        TP_y_sol = value(TP_y);
        TS_y_sol = value(TS_y);

        if TP_x_sol >= 0
            TP_cmd = sqrt(TP_x_sol*TP_x_sol + TP_y_sol*TP_y_sol);
        else 
            TP_cmd = -sqrt(TP_x_sol*TP_x_sol + TP_y_sol*TP_y_sol);
        end

        if TS_x_sol >= 0
            TS_cmd = sqrt(TS_x_sol*TS_x_sol + TS_y_sol*TS_y_sol);
        else 
            TS_cmd = -sqrt(TS_x_sol*TS_x_sol + TS_y_sol*TS_y_sol);
        end

        delPR_cmd = atan2(-TP_y_sol,TP_x_sol);
        delSR_cmd = atan2(-TS_y_sol,TS_x_sol);
        

end
