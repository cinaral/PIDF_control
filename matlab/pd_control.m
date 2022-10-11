function [y_next] = pd_control(T_s, T_f, K_p, K_d, x, x_next, y)
%* cinaral 2022-10-10
persistent common_den;
persistent coef_y;
persistent coef_x;
persistent coef_x_next;

if isempty(common_den)
    common_den = (2*T_f + T_s);
    coef_y = (2*T_f - T_s)/common_den;
    coef_x = -(2*K_d + 2*K_p*T_f - K_p*T_s)/common_den;
    coef_x_next = (2*K_d + 2*K_p*T_f + K_p*T_s)/common_den;
end

y_next = coef_x*x + coef_x_next*x_next + coef_y*y;
