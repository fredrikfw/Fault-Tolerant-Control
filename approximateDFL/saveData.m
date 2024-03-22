t_s = 0:0.04:t(length(t));

state_extended = zeros(length(t),18);
state_extended(:,1:6) = state(:,1:6);
state_extended(:,16:18) = state(:,10:12);

R0 = [-1 0 0; 0 -1 0; 0 0 1];

for i=1:length(t)
    R = R0*rpy(state(i,7),state(i,8),state(i,9));
    state_extended(i,7:9) = R(:,1);
    state_extended(i,10:12) = R(:,2);
    state_extended(i,13:15) = R(:,3);
end
state_s = interp1(t,state_extended,t_s);
state_s(:,3) = state_s(:,3)+3;
save('/home/pao/dev/ros_workspace/matlab2quad/state.txt','state_s','-ascii');