function x = init_navigation_state(~, settings)

roll = 0;
pitch = 0;

% Initial coordinate rotation matrix
q = ch_eul2q([roll pitch settings.init_heading]);
x = [zeros(6,1); q];

end