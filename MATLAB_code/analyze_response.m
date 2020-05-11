function [t_settle, t_rise, overshoot, steady_state_error] = analyze_response(tout, y, y_final, flag_step)
% Compute step responses give time (tout), response (y), steady state
% (y_final).
% flag_step is 1 if the response is a step response.
% flag_step is 0 if the resposne is NOT a step response.


if flag_step == 1
    % Compute rise time (t_rise)
    i_90 = find(abs(y - 0.9 * y_final) < 0.01);
    i_10 = find(abs(y - 0.1 * y_final) < 0.01);

    t_90 = tout(i_90(1));
    t_10 = tout(i_10(1));

    t_rise = t_90 - t_10;
elseif flag_step == 0
    t_rise = 0;
end


% Compute overshoot 
[peak, peak_loc] = max(y);
overshoot = (peak - y_final);



% compute steady state error
steady_state_error = (y(end) - y_final);



% Compute settle time (t_settle)
if flag_step == 1
    non_step_bias = 0;
elseif flag_step == 0 
   non_step_bias = 0.05; 
end


for i = length(y):-1:1
    if (abs(y(i) - y_final)) > (0.05 * y_final + non_step_bias)
        i_settle = i;
        break
    else
        i_settle = 1;
    end
    
end

t_settle = tout(i_settle);



end
