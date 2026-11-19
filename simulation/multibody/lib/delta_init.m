function delta_init(root)
%DELTA_INIT  Called by the model (InitFcn): puts circ and geom in the base workspace.
if nargin < 1, root = fileparts(fileparts(mfilename('fullpath'))); end
circ = params_delta();
S = load(fullfile(root,'results','geometry.mat'));
assignin('base','circ',circ);
assignin('base','geom',S.geom);
end
