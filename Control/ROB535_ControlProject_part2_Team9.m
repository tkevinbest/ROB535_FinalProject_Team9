function [sol_2, FLAG_terminate] = ROB535_ControlProject_part2_Team9(TestTrack, Xobs_seen, curr_state)
% ROB535_ControlProject_part2_Team9
%%% Inputs
%   TestTrack
%   Xobs_seen
%   curr_state
%%% Outputs
%   sol_2 - Vector of control inputs which will be passed to forwardIntegrateControlInput
%           Must have enough control inputs with time step 0.01 for 0.5
%           seconds
%   FLAG_terminate - binary flag to stop simulation

dt = 0.01;
sol_2 = ones(0.5/dt,2);

FLAG_terminate = true;

end