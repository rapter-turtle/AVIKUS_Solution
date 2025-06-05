function [Fx, Fy] = cbf(Fx0, Fy0, x, y, x_dot, y_dot, alpha1, alpha2, alpha_l, alpha_r, a, control_update_dt)
        
        f_max = 0.3;
        w_max = 0.2;
        ux_opt = sdpvar(1,1);      
        uy_opt = sdpvar(1,1);
        s1 = sdpvar(1,1);
        s2 = sdpvar(1,1);

        %% Define Cost Function
        Objective = (ux_opt - Fx0)^2 + (uy_opt - Fy0)^2 + 1e9 * s1^2 + 1e9 * s2^2;

        %% Define Constraints (CBF)
        Constraints = [];
       
        %% Nominal
        % Constraints = [Constraints, (alpha1 + alpha2)*(a*x_dot + y_dot) + alpha1*alpha2*(a*x - y) + (a*ux_opt + uy_opt) + s1 >= 0];
        % Constraints = [Constraints, (alpha1 + alpha2)*(-a*x_dot + y_dot) + alpha1*alpha2*(-a*x - y) + (-a*ux_opt + uy_opt) + s2 >= 0];
        
        %% Nominal Robust
        % Constraints = [Constraints, (alpha1 + alpha2)*(a*x_dot + y_dot) + alpha1*alpha2*(a*x - y) + (a*ux_opt + uy_opt) + (a*w_max + w_max) + s1 >= 0];
        % Constraints = [Constraints, (alpha1 + alpha2)*(-a*x_dot + y_dot) + alpha1*alpha2*(-a*x - y) + (-a*ux_opt + uy_opt) + (a*w_max + w_max) + s2 >= 0];
              
        %% fea_Nominal
        % Constraints = [Constraints, (alpha1 + alpha2)*(a*ux_opt - uy_opt) + alpha1*alpha2*(a*x_dot - y_dot) + alpha_l*((alpha1 + alpha2)*(a*x_dot + y_dot) + alpha1*alpha2*(a*x - y) - (-a*f_max - f_max)) + s1 >= 0];
        % Constraints = [Constraints, (alpha1 + alpha2)*(-a*ux_opt - uy_opt) + alpha1*alpha2*(-a*x_dot - y_dot) + alpha_r*((alpha1 + alpha2)*(-a*x_dot + y_dot) + alpha1*alpha2*(-a*x - y) - (-a*f_max - f_max)) + s2 >= 0];
         
        %% fea_RaCBF
        % Constraints = [Constraints, (alpha1 + alpha2)*(a*ux_opt - uy_opt) + alpha1*alpha2*(a*x_dot(i-1) - y_dot(i-1)) + (a*x_dis_dot - y_dis_dot) - (a*DOB_bound + DOB_bound) + alpha_l*((alpha1 + alpha2)*(a*x_dot(i-1) + y_dot(i-1)) + alpha1*alpha2*(a*x(i-1) - y(i-1)) + (-a*f_max - f_max) + (a*x_dis_estim(i-1) - y_dis_estim(i-1))) + s1 >= 0];
        % Constraints = [Constraints, (alpha1 + alpha2)*(-a*ux_opt - uy_opt) + alpha1*alpha2*(-a*x_dot(i-1) - y_dot(i-1)) + (-a*x_dis_dot - y_dis_dot) - (a*DOB_bound + DOB_bound) + alpha_r*((alpha1 + alpha2)*(-a*x_dot(i-1) + y_dot(i-1)) + alpha1*alpha2*(-a*x(i-1) - y(i-1)) + (-a*f_max - f_max) + (-a*x_dis_estim(i-1) - y_dis_estim(i-1))) + s2 >= 0];
        % 
        %% fea_RCBF
        Constraints = [Constraints, (alpha1 + alpha2)*(a*ux_opt - uy_opt) + alpha1*alpha2*(a*x_dot - y_dot) - (a*w_max + w_max) + alpha_l*((alpha1 + alpha2)*(a*x_dot + y_dot) + alpha1*alpha2*(a*x - y) - (-a*f_max - f_max) - (a*w_max + w_max)) + s1 >= 0];
        Constraints = [Constraints, (alpha1 + alpha2)*(-a*ux_opt - uy_opt) + alpha1*alpha2*(-a*x_dot - y_dot) - (a*w_max + w_max) + alpha_r*((alpha1 + alpha2)*(-a*x_dot + y_dot) + alpha1*alpha2*(-a*x - y) - (-a*f_max - f_max) - (a*w_max + w_max)) + s2 >= 0];

        %% fixed fea_RCBF
        % Constraints = [Constraints, (alpha1 + alpha2)*(a*ux_opt - uy_opt) + alpha1*alpha2*(a*x_dot(i-1) - y_dot(i-1)) -(a*w_max + w_max) + alpha_l*((alpha1 + alpha2)*(a*x_dot(i-1) + y_dot(i-1)) + alpha1*alpha2*(a*x(i-1) - y(i-1)) + (-a*f_max - f_max) - (a*w_max + w_max)) + s1 >= 0];
        % Constraints = [Constraints, (alpha1 + alpha2)*(-a*ux_opt - uy_opt) + alpha1*alpha2*(-a*x_dot(i-1) - y_dot(i-1)) -(a*w_max + w_max) + alpha_r*((alpha1 + alpha2)*(-a*x_dot(i-1) + y_dot(i-1)) + alpha1*alpha2*(-a*x(i-1) - y(i-1)) + (-a*f_max - f_max) - (-a*w_max + w_max)) + s2 >= 0];
        % Constraints = [Constraints, uy_opt >= a*ux_opt + (-a*f_max - f_max) + (a*w_max + w_max)];
        % Constraints = [Constraints, uy_opt >= -a*ux_opt + (-a*f_max - f_max) + (a*w_max + w_max)];
        % Constraints = [Constraints, uy_opt <= a*ux_opt - (-a*f_max - f_max) - (a*w_max + w_max)];
        % Constraints = [Constraints, uy_opt <= -a*ux_opt - (-a*f_max - f_max) - (a*w_max + w_max)];

        %% Define Constraints (ux, uy)
        Constraints = [Constraints, ux_opt <= f_max];
        Constraints = [Constraints, ux_opt >= -f_max];
        Constraints = [Constraints, uy_opt <= f_max];
        Constraints = [Constraints, uy_opt >= -f_max];
        
        % Constraints = [Constraints, ux_opt <= Fx0 + 0.5*control_update_dt*f_max];
        % Constraints = [Constraints, ux_opt >= Fx0 - 0.5*control_update_dt*f_max];
        % Constraints = [Constraints, uy_opt <= Fy0 + 0.5*control_update_dt*f_max];
        % Constraints = [Constraints, uy_opt >= Fy0 - 0.5*control_update_dt*f_max];

        %% Solve using SDPT3
        options = sdpsettings('solver', 'sdpt3', 'verbose', 0);
        sol = optimize(Constraints, Objective, options);

        Fx = value(ux_opt);
        Fy = value(uy_opt);
end
