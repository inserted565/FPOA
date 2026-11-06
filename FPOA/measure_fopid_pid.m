%% measure_fopid_pid.m
% Calculates performance metrics for FOPID and PID controllers in fopid.slx.
%
% Metric Definitions:
%   IAE             : Integral of absolute error, calculated per segment and total (0-10s).
%   Peak Overshoot  : Max peak beyond target in the step direction / |Step Amplitude| * 100%
%   Peak Undershoot : Max opposite peak after reaching the target / |Step Amplitude| * 100%
%   Settling Time   : Last time leaving the target ±2% band (relative to segment start).
%
% Reference signal is the 3-step sequence: 0→0.6 (0s), 0.6→0.25 (3s), 0.25→0.75 (7s).
% Data is read from the 3 channels in the model's Scope:
%   Ch 1 = FOPID theta, Ch 2 = PID theta, Ch 3 = Reference.
%
% Outputs generated in the same folder:
%   fopid_pid_metrics.csv / .mat
%   fopid_pid_tracking.png
%   fopid_pid_metrics.png

clear; clc; close all;

%% 1. Locate model
scriptDir = fileparts(mfilename('fullpath'));
mdlPath = fullfile(scriptDir, 'fopid.slx');
assert(isfile(mdlPath), 'Cannot find fopid.slx. Please place this script in the same folder.');

modelDir = fileparts(mdlPath);
oldDir = pwd;
cd(modelDir);
cleanupDir = onCleanup(@() cd(oldDir));

%% 2. Simulation
if bdIsLoaded('fopid')
    close_system('fopid', 0);       
end
load_system('fopid.slx');
stopTime = get_param('fopid', 'StopTime');

fprintf('Simulating fopid.slx (StopTime = %s s)...\n', stopTime);
out = sim('fopid', 'StopTime', stopTime, ...
    'SaveOutput', 'on', 'ReturnWorkspaceOutputs', 'on');

%% 3. Read Scope data
try
    sd = out.scope_data;
catch
    sd = evalin('base', 'scope_data');
end

t = sd.time(:);
nSig = numel(sd.signals);
Y = zeros(numel(t), nSig);
for k = 1:nSig
    v = sd.signals(k).values;
    if isa(v, 'timeseries')
        v = v.Data;
    end
    Y(:, k) = v(:);
end
if nSig < 3
    error('Scope must have at least 3 signals: FOPID theta, PID theta, Reference.');
end

if t(1) > 0
    t = [0; t];
    Y = [Y(1, :); Y];
end

thetaF = Y(:, 1);   
thetaP = Y(:, 2);   
ref    = Y(:, 3);   

%% 4. Identify step segments automatically
changeIdx = find(abs(diff(ref)) > 1e-6) + 1;
segStart = [1; changeIdx(:)];
segEnd   = [changeIdx(:) - 1; numel(t)];
valid    = segEnd >= segStart;
segStart = segStart(valid);
segEnd   = segEnd(valid);

nSeg = numel(segStart);
targets = ref(segEnd);
prevTargets = zeros(nSeg, 1);
if nSeg > 1
    prevTargets(2:end) = ref(segStart(2:end) - 1);
end

segLabels = strings(nSeg, 1);
for k = 1:nSeg
    segLabels(k) = sprintf('%.0f-%.0f s', t(segStart(k)), t(segEnd(k)));
end

%% 5. Calculate metrics per segment
mF = repmat(empty_metrics(), nSeg, 1);
mP = repmat(empty_metrics(), nSeg, 1);
for k = 1:nSeg
    idx = segStart(k):segEnd(k);
    mF(k) = segment_metrics(t(idx), thetaF(idx), ref(idx), prevTargets(k), targets(k));
    mP(k) = segment_metrics(t(idx), thetaP(idx), ref(idx), prevTargets(k), targets(k));
end

iaeF = sum([mF.IAE]);   
iaeP = sum([mP.IAE]);

%% 6. Summarize and print table
Controller = [repmat("FOPID", nSeg, 1); repmat("PID", nSeg, 1)];
Segment    = [segLabels; segLabels];
Target     = [targets; targets];
IAE        = [[mF.IAE]'; [mP.IAE]'];
OS         = [[mF.Overshoot]'; [mP.Overshoot]'];
US         = [[mF.Undershoot]'; [mP.Undershoot]'];
Ts         = [[mF.SettlingTime]'; [mP.SettlingTime]'];
Peak       = [[mF.Peak]'; [mP.Peak]'];
PeakTime   = [[mF.PeakTime]'; [mP.PeakTime]'];

T = table(Controller, Segment, Target, IAE, OS, US, Ts, Peak, PeakTime, ...
    'VariableNames', {'Controller', 'Segment', 'Target', 'IAE', ...
    'Overshoot_pct', 'Undershoot_pct', 'SettlingTime_s', 'Peak', 'PeakTime_s'});

fprintf('\n%s\n', repmat('=', 1, 92));
fprintf('FOPID / PID Performance Metrics (Ref: 0->0.6 @0s, 0.6->0.25 @3s, 0.25->0.75 @7s)\n');
fprintf('%s\n', repmat('=', 1, 92));
fprintf('%-7s %-9s %7s %10s %10s %10s %13s\n', ...
    'Ctrl', 'Segment', 'Target', 'IAE', 'OS(%)', 'US(%)', 'Ts(2%)');
fprintf('%s\n', repmat('-', 1, 92));
for r = 1:height(T)
    if isnan(T.SettlingTime_s(r))
        tsStr = '--';
    else
        tsStr = sprintf('%.3f', T.SettlingTime_s(r));
    end
    fprintf('%-7s %-9s %7.3f %10.5f %10.2f %10.2f %13s\n', ...
        char(T.Controller(r)), char(T.Segment(r)), T.Target(r), ...
        T.IAE(r), T.Overshoot_pct(r), T.Undershoot_pct(r), tsStr);
end
fprintf('%s\n', repmat('-', 1, 92));
fprintf('Total IAE (0~%g s): FOPID = %.5f, PID = %.5f', t(end), iaeF, iaeP);
if iaeP > 0
    fprintf(' (FOPID relative change %+.2f%%)\n', 100 * (iaeF - iaeP) / iaeP);
else
    fprintf('\n');
end

kFinal = nSeg;
fprintf('\nFinal step (%.2f -> %.2f):\n', prevTargets(kFinal), targets(kFinal));
fprintf('  FOPID: OS = %6.2f%%, US = %6.2f%%, Ts = %.3f s\n', ...
    mF(kFinal).Overshoot, mF(kFinal).Undershoot, mF(kFinal).SettlingTime);
fprintf('  PID  : OS = %6.2f%%, US = %6.2f%%, Ts = %.3f s\n', ...
    mP(kFinal).Overshoot, mP(kFinal).Undershoot, mP(kFinal).SettlingTime);

Summary = table(["FOPID"; "PID"], [iaeF; iaeP], ...
    [max([mF.Overshoot]); max([mP.Overshoot])], ...
    [max([mF.Undershoot]); max([mP.Undershoot])], ...
    [max([mF.SettlingTime], [], 'omitnan'); max([mP.SettlingTime], [], 'omitnan')], ...
    'VariableNames', {'Controller', 'IAE_Total', 'Max_Overshoot_pct', ...
    'Max_Undershoot_pct', 'Max_SettlingTime_s'});
disp(Summary);

%% 7. Plotting
set(0, 'DefaultAxesFontName', 'Arial');

fig1 = figure('Color', 'w', 'Name', 'FOPID vs PID tracking', ...
    'Position', [80, 80, 1000, 700]);
subplot(2, 1, 1);
plot(t, ref, 'k--', 'LineWidth', 1.2); hold on;
plot(t, thetaF, 'b-', 'LineWidth', 1.5);
plot(t, thetaP, 'r-', 'LineWidth', 1.5);
grid on; box on;
xlim([0, t(end)]);
xlabel('Time t (s)'); ylabel('\theta (rad)');
legend({'Reference', 'FOPID', 'PID'}, 'Location', 'best');
title('Tracking Response of FOPID and PID');
for k = 2:nSeg
    xline(t(segStart(k)), 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
end

subplot(2, 1, 2);
plot(t, ref - thetaF, 'b-', 'LineWidth', 1.2); hold on;
plot(t, ref - thetaP, 'r-', 'LineWidth', 1.2);
yline(0, 'k-', 'LineWidth', 0.8);
grid on; box on;
xlim([0, t(end)]);
xlabel('Time t (s)'); ylabel('Tracking Error e (rad)');
legend({'FOPID', 'PID'}, 'Location', 'best');
title('Tracking Error Comparison');

fig2 = figure('Color', 'w', 'Name', 'FOPID vs PID metrics', ...
    'Position', [120, 120, 1050, 720]);
IAE_mat = [[mF.IAE]', [mP.IAE]'];
OS_mat  = [[mF.Overshoot]', [mP.Overshoot]'];
US_mat  = [[mF.Undershoot]', [mP.Undershoot]'];
Ts_mat  = [[mF.SettlingTime]', [mP.SettlingTime]'];
mats   = {IAE_mat, OS_mat, US_mat, Ts_mat};
titles = {'IAE', 'Peak Overshoot (%)', 'Peak Undershoot (%)', 'Settling Time (s, \pm2%)'};
for k = 1:4
    subplot(2, 2, k);
    bar(1:nSeg, mats{k});
    grid on; box on;
    set(gca, 'XTick', 1:nSeg, 'XTickLabel', cellstr(segLabels));
    xlabel('Step Segment'); ylabel(titles{k});
    title(titles{k});
    if k == 1
        legend({'FOPID', 'PID'}, 'Location', 'best');
    end
end
sgtitle('FOPID and PID Performance Metrics Comparison');

%% 8. Save results
csvPath = fullfile(modelDir, 'fopid_pid_metrics.csv');
matPath = fullfile(modelDir, 'fopid_pid_metrics.mat');
fig1Path = fullfile(modelDir, 'fopid_pid_tracking.png');
fig2Path = fullfile(modelDir, 'fopid_pid_metrics.png');

writetable(T, csvPath, 'Encoding', 'UTF-8');
save(matPath, 'T', 'Summary', 't', 'Y', 'mF', 'mP', 'iaeF', 'iaeP', '-v7.3');
exportgraphics(fig1, fig1Path, 'Resolution', 200);
exportgraphics(fig2, fig2Path, 'Resolution', 200);

fprintf('\nResults saved to:\n  %s\n  %s\n  %s\n  %s\n', ...
    csvPath, matPath, fig1Path, fig2Path);

%% ====================== Local Functions ======================
function m = empty_metrics()
m = struct('IAE', NaN, 'Overshoot', NaN, 'Undershoot', NaN, ...
    'SettlingTime', NaN, 'Peak', NaN, 'PeakTime', NaN, ...
    'Final', NaN, 'SSerror', NaN, 'Target', NaN, 'Amp', NaN, 'Band', NaN);
end

function m = segment_metrics(ts, ys, rs, rPrev, rTarget)
% Metrics for a single step segment. OS/US are defined by step direction, divided by |Δr|.
ts = ts(:); ys = ys(:); rs = rs(:);
m = empty_metrics();
m.Target = rTarget;
A = rTarget - rPrev;          
m.Amp = A;
m.IAE = trapz(ts, abs(rs - ys));

if numel(ts) < 2 || abs(A) < 1e-12
    return;
end

d = sign(A);                  
if d > 0
    [ypeak, ipk] = max(ys);
else
    [ypeak, ipk] = min(ys);
end
m.Peak = ypeak;
m.PeakTime = ts(ipk) - ts(1);

m.Overshoot = max(0, d * (ypeak - rTarget)) / abs(A) * 100;

if d > 0
    yext = min(ys(ipk:end));
else
    yext = max(ys(ipk:end));
end
m.Undershoot = max(0, -d * (yext - rTarget)) / abs(A) * 100;

band = 0.02 * abs(rTarget);
if band < 1e-12
    band = 0.02 * abs(A);
end
m.Band = band;
outside = find(abs(ys - rTarget) > band, 1, 'last');
if isempty(outside)
    m.SettlingTime = 0;
elseif outside == numel(ys)
    m.SettlingTime = NaN;     
else
    m.SettlingTime = ts(outside) - ts(1);
end

m.Final = mean(ys(max(1, end - 49):end));
m.SSerror = rTarget - m.Final;
end