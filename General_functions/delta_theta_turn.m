function delta_th = delta_theta_turn(th_initial, th_final, direction)
% this function returns the angle difference in the [-2pi, 2pi] range

if strcmpi( direction, 'cw' ) % check if the direction is clockwise
    th_init_0_2pi = mod(th_initial, 2*pi) ; % here we shift the "th_initial"to the [0,2pi] range
    th_fin_0_2pi = mod(th_final, 2*pi) ; % here we shift the "th_final"to the [0,2pi] range
    if th_init_0_2pi >= th_fin_0_2pi % This means we do not pass the zero in  going from "th_init_0_2pi" to "th_fin_0_2pi" in the cw direction.
        delta_th = th_fin_0_2pi - th_init_0_2pi;
    else % This means we have to pass the zero in  going from "th_init_0_2pi" to "th_fin_0_2pi". Therefore, in this case, we first go to zero from "th_init_0_2pi" and then from zero we go to the "th_fin_0_2pi".
        th_fin_negative2pi_0 = th_fin_0_2pi - 2*pi; % here we shift the "th_final" to the [-2pi,0] range
        delta_th = th_fin_negative2pi_0 - th_init_0_2pi;
    end
elseif strcmpi( direction, 'ccw' ) % check if the direction is counterclockwise
    th_init_0_2pi = mod(th_initial, 2*pi) ; % here we shift the "th_initial"to the [0,2pi] range
    th_fin_0_2pi = mod(th_final, 2*pi) ; % here we shift the "th_final"to the [0,2pi] range
    if th_init_0_2pi <= th_fin_0_2pi % This means we do not pass the zero in  going from "th_init_0_2pi" to "th_fin_0_2pi" in the ccw direction.
        delta_th = th_fin_0_2pi - th_init_0_2pi;
    else % This means we have to pass the zero in  going from "th_init_0_2pi" to "th_fin_0_2pi". Therefore, in this case, we first go to zero from "th_init_0_2pi" and then from zero we go to the "th_fin_0_2pi".
        th_init_negative2pi_0 = th_init_0_2pi - 2*pi; % here we shift the "th_init" to the [-2pi,0] range
        delta_th = th_fin_0_2pi - th_init_negative2pi_0;
    end
else
    error('The direction must be either "cw" (clockwise) or "ccw" (counterclockwise)')
end

end