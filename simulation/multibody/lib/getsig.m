function [t, D] = getsig(out, name)
%GETSIG  Read a To Workspace timeseries from a Simulink.SimulationOutput.
%   [t, D] = getsig(out, name)  -> t: Nx1 [s], D: NxM
try
    ts = out.get(name);
catch
    ts = [];
end
if isempty(ts)
    error('Signal "%s" not found in simulation output. Did run_01_build_model finish?', name);
end
t = ts.Time(:);
D = squeeze(ts.Data);
if size(D,1) ~= numel(t), D = D.'; end
end
