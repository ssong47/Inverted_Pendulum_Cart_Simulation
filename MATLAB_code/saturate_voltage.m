function r_new = saturate_voltage(input_voltage, p, K, Nbar, X)

voltage_soft_limit = p.voltage_soft_limit;
ov_counter_limit = p.ov_counter_limit;
ov_absolute_limit = p.ov_absolute_limit;

global ov_counter

if (input_voltage > ov_absolute_limit)
    real_input_voltage = ov_absolute_limit;

    
    
elseif (input_voltage < -ov_absolute_limit)
    real_input_voltage = -ov_absolute_limit;

    
    
    
else
    if (input_voltage > voltage_soft_limit)
       ov_counter = ov_counter + 1;
       if (ov_counter > ov_counter_limit)
            real_input_voltage = voltage_soft_limit;  
            
         
       else
           real_input_voltage = input_voltage;
       end
       
       
       
    elseif (input_voltage < -voltage_soft_limit)
       ov_counter = ov_counter + 1;
       if (ov_counter > ov_counter_limit)
            real_input_voltage = -voltage_soft_limit;
           
       else
           real_input_voltage = input_voltage;
       end
       
       
       
    else
       real_input_voltage = input_voltage;
       ov_counter = 0;
    end
end


if p.flag_ctrl == 0
    r_new = (real_input_voltage + K * X)/Nbar;
elseif p.flag_ctrl == 1
    r_new = real_input_voltage;
end


end