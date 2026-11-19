%% STEP4_VIDEO  Export the last simulation from Mechanics Explorer as .mp4
% Before running: in Mechanics Explorer choose the view you like (e.g. "Y up" then "Iso").
addpath(fullfile(fileparts(mfilename('fullpath')),'lib'));
cfg = v2_config();
outDir = fullfile(cfg.resultsDir,'circle'); if ~exist(outDir,'dir'), mkdir(outDir); end
smwritevideo(cfg.mdl, fullfile(outDir,'delta_circle'), 'PlaybackSpeedRatio',1, ...
    'FrameRate',30, 'VideoFormat','mpeg-4');
fprintf('Saved %s\n', fullfile(outDir,'delta_circle.mp4'));
