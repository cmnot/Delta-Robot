function cfg = v2_config()
%V2_CONFIG  Folders and model names (normally nothing to edit here)
cfg.root       = fileparts(fileparts(mfilename('fullpath')));      % ...\simulation\multibody
cfg.modelDir   = fullfile(cfg.root, 'model');
cfg.resultsDir = fullfile(cfg.root, 'results');
cfg.origModel  = 'Multibody_Deltarobot';
cfg.mdl        = 'DeltaRobot_V2';          % multibody model with the MATLAB Function actuator (model/)
cfg.jointNames = {'Revolute Joint','Revolute Joint1','Revolute Joint2'};
cfg.convNames  = {'Simulink-PS Converter','Simulink-PS Converter1','Simulink-PS Converter2'};
cfg.armNames   = {'link 1','link 2','link 3'};
cfg.effName    = 'Subsystem';
cfg.worldName  = 'World Frame';
cfg.q_home_deg = -80.59845582369277;       % value of the Constant blocks in the original model
% limits used only for the workspace plot (same as Claude_helper V1)
cfg.beta_min_deg = -30;  cfg.beta_max_deg = 90;  cfg.ball_alpha_max_deg = 35;
end
