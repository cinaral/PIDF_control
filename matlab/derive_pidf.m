%* y_next = x_coef*x + y_coef*y
syms s z K_p K_i K_d T_s T_f;

s_bilinear_approx = 2 / T_s * (z - 1) / (z + 1);

H_DF = s / (T_f * s + 1);
[DF_num, DF_den] = numden((subs(H_DF, s, s_bilinear_approx)));
DF_num_coeffs = coeffs(DF_num, z);
DF_den_coeffs = coeffs(DF_den, z);
DF_coef_x = DF_num_coeffs./DF_den_coeffs(2);
DF_coef_y = -DF_den_coeffs(1)./DF_den_coeffs(2);

H_PDF = K_p + K_d * s / (T_f * s + 1);
[PDF_num, PDF_den] = numden((subs(H_PDF, s, s_bilinear_approx)));
PDF_num_coeffs = coeffs(PDF_num, z);
PDF_den_coeffs = coeffs(PDF_den, z);
PDF_coef_x = PDF_num_coeffs./PDF_den_coeffs(2)
PDF_coef_y = -PDF_den_coeffs(1)./PDF_den_coeffs(2)

H_PI = K_p + K_i / s;
[PI_num, PI_den] = numden((subs(H_PI, s, s_bilinear_approx)));
PI_num_coeffs = coeffs(PI_num, z);
PI_den_coeffs = coeffs(PI_den, z);
PI_coef_x = PI_num_coeffs./PI_den_coeffs(2);
PI_coef_y = -PI_den_coeffs(1)./PI_den_coeffs(2);

H_PIDF = K_p +  K_i / s + K_d * s / (T_f * s + 1);
[PIDF_num, PIDF_den] = numden((subs(H_PIDF, s, s_bilinear_approx)));
PIDF_num_coeffs = coeffs(PIDF_num, z);
PIDF_den_coeffs = coeffs(PIDF_den, z);
PIDF_coef_x = PIDF_num_coeffs./PIDF_den_coeffs(3);
PIDF_coef_y = -PIDF_den_coeffs(1:2)./PIDF_den_coeffs(3);