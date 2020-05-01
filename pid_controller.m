function u = pid_controller(error_hist)
    % controller 
    Kp = 2.7754; %proportional gain
    Ki = 2.1846 ;
    Kd = 20.0446 ;
    u_steady = 0;
    u = u_steady + (Kp * (error_hist(end))) + ...
        (Ki * sum(error_hist)) +...
        (Kd * (error_hist(end) - error_hist(end-1)));
end


