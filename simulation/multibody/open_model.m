%% OPEN_MODEL  Open the delta robot multibody model (adds the needed folders to the path)
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'model'), fullfile(root,'lib'), root);
open_system(fullfile(root,'model','DeltaRobot_V2.slx'));
% Press Run in Simulink. The motion is set in params_delta.m.
