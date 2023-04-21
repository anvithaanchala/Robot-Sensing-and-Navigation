% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1543.107520876155604 ; 1540.972401489566664 ];

%-- Principal point:
cc = [ 577.346465418866842 ; 1029.983799469696351 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.159911602111241 ; -0.478725754491784 ; -0.000016013620522 ; 0.000035135384716 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 22.838550922148556 ; 22.986266402539567 ];

%-- Principal point uncertainty:
cc_error = [ 3.586417507353713 ; 5.371142013369227 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.009118592408099 ; 0.046672331702047 ; 0.001188267913746 ; 0.000764466231578 ; 0.000000000000000 ];

%-- Image size:
nx = 1152;
ny = 2048;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 10;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.859773e+00 ; 4.048130e-01 ; -9.489089e-02 ];
Tc_1  = [ -8.307469e+01 ; -8.185249e+00 ; 3.591307e+02 ];
omc_error_1 = [ 4.223528e-03 ; 1.136596e-03 ; 3.730653e-03 ];
Tc_error_1  = [ 8.284265e-01 ; 1.254239e+00 ; 5.306003e+00 ];

%-- Image #2:
omc_2 = [ 2.897210e+00 ; 9.873467e-01 ; -3.771976e-02 ];
Tc_2  = [ -9.627146e+01 ; -5.092125e+01 ; 4.411118e+02 ];
omc_error_2 = [ 3.761321e-03 ; 1.624051e-03 ; 5.302414e-03 ];
Tc_error_2  = [ 1.025637e+00 ; 1.536265e+00 ; 6.545013e+00 ];

%-- Image #3:
omc_3 = [ 3.021216e+00 ; -5.815391e-01 ; 7.113752e-02 ];
Tc_3  = [ -1.850175e+01 ; 7.128968e+01 ; 3.563918e+02 ];
omc_error_3 = [ 3.340893e-03 ; 1.006268e-03 ; 4.857740e-03 ];
Tc_error_3  = [ 8.318461e-01 ; 1.252331e+00 ; 5.284276e+00 ];

%-- Image #4:
omc_4 = [ 3.127426e+00 ; 6.590117e-02 ; -2.048768e-02 ];
Tc_4  = [ -7.561962e+01 ; 4.729402e+01 ; 3.382480e+02 ];
omc_error_4 = [ 3.158384e-03 ; 6.054076e-04 ; 4.764076e-03 ];
Tc_error_4  = [ 7.836984e-01 ; 1.182450e+00 ; 5.018209e+00 ];

%-- Image #5:
omc_5 = [ 3.102816e+00 ; -3.432554e-02 ; 7.516122e-02 ];
Tc_5  = [ -6.055882e+01 ; 8.171504e+01 ; 2.887428e+02 ];
omc_error_5 = [ 3.019433e-03 ; 4.931580e-04 ; 4.087020e-03 ];
Tc_error_5  = [ 6.792211e-01 ; 1.017847e+00 ; 4.277067e+00 ];

%-- Image #6:
omc_6 = [ 2.926871e+00 ; 4.932814e-01 ; -3.692233e-02 ];
Tc_6  = [ -8.973912e+01 ; 3.861926e+01 ; 3.583479e+02 ];
omc_error_6 = [ 3.834713e-03 ; 9.556983e-04 ; 4.298132e-03 ];
Tc_error_6  = [ 8.344463e-01 ; 1.255873e+00 ; 5.306021e+00 ];

%-- Image #7:
omc_7 = [ 2.932733e+00 ; 1.355056e-01 ; -1.391367e-01 ];
Tc_7  = [ -7.060542e+01 ; 3.737980e+01 ; 3.074776e+02 ];
omc_error_7 = [ 3.930441e-03 ; 8.226472e-04 ; 3.697334e-03 ];
Tc_error_7  = [ 7.061590e-01 ; 1.078983e+00 ; 4.497102e+00 ];

%-- Image #8:
omc_8 = [ -3.008709e+00 ; -7.621277e-03 ; 1.113951e-02 ];
Tc_8  = [ -6.706396e+01 ; 6.576851e+01 ; 3.167004e+02 ];
omc_error_8 = [ 3.423457e-03 ; 5.566033e-04 ; 4.035414e-03 ];
Tc_error_8  = [ 7.275362e-01 ; 1.077629e+00 ; 4.600360e+00 ];

%-- Image #9:
omc_9 = [ 3.124533e+00 ; 1.041798e-02 ; -3.086720e-03 ];
Tc_9  = [ -6.429861e+01 ; 3.830922e+01 ; 2.906868e+02 ];
omc_error_9 = [ 2.986739e-03 ; 5.845027e-04 ; 4.287280e-03 ];
Tc_error_9  = [ 6.696865e-01 ; 1.018451e+00 ; 4.311513e+00 ];

%-- Image #10:
omc_10 = [ 2.860959e+00 ; -4.898609e-01 ; 1.935227e-01 ];
Tc_10  = [ -2.312412e+01 ; 4.428392e+01 ; 3.039180e+02 ];
omc_error_10 = [ 4.024300e-03 ; 1.291673e-03 ; 3.746310e-03 ];
Tc_error_10  = [ 7.049738e-01 ; 1.061019e+00 ; 4.564044e+00 ];

