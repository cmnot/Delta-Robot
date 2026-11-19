function f = newfig(name)
%NEWFIG  Figure with the light theme (report-friendly, also when MATLAB is in dark mode).
f = figure('Name', name, 'Color', 'w');
try, theme(f, 'light'); catch, end
end
