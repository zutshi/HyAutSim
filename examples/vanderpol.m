%% From Xin's examples: http://www-i2.informatik.rwth-aachen.de/i2/fileadmin/user_upload/documents/HybridSystemsGroup/chen/benchmarks/Lorentz_short.model

% function ret_val = vanderpol(get_structs)
function [sys_def,sys_prop,sys_opt] = vanderpol()

%% sys info
sys_def.NUM_STATE_VARS = NUM_STATE_VARS;
sys_def.NUM_CINPUT_VARS = 0;
sys_def.NUM_PARAMS = 0;
sys_def.MODE_LIST = [1];


sys_def.hybrid = 0;

%% functions
sys_def.transitionMap = getTransitionMap(sys_def);
sys_def.modeInvMap = getModeInvMap();
sys_def.modeDynMap = getModeDynMap();
%sys_maps.getModeTransitions = getModeTransitionMap();

%sys_maps.maxDwellTimesMap = getMaxDwellTimesMap();
%sys_maps.minDwellTimesMap = getMinDwellTimesMap();

%sys_maps.maxStateVarsMap = getMaxStateVarsMap();
%sys_maps.minStateVarsMap = getMinStateVarsMap();

sys_prop.initConstraints = initConstraints();
sys_prop.finalConstraints = finalConstraints();

sys_opt.plotFun = @plotFun;
sys_opt.initFun = @initFun;
sys_opt.drawProp = @drawProp;
sys_opt.useGrad = 0;

%% sim-params
sys_opt.samplingType = 'UniformRand';%['UniformAll' 'Halton']
%sys_opt.minLengthTrajThresh = 0.5;
sys_opt.GuardDetectRoundDigits = 4;

%% Horizon
%sys_opt.ScatterSim.MAX_SAMPLE_AGE = 1;%0.7;
sys_opt.ScatterSim.MAX_SWITCHINGS = inf;
%sys_opt.MODE_TIME_HORIZON = 0.3;%1;%0.7;

sys_def.str = 'vanderpol';

sim_fn = hybrid_system_simulator(sys_def,sys_prop,sys_opt);
delta = 0.01;
TH = 1

%% test simulator
figure(1)
hold on
t = [0];

N = 10;
cons = initConstraints()
X0_set = cons.cube;
X0_samples = genRandVectors(rand(N, NUM_STATE_VARS), X0_set);

for i = 1:N
    XArr = X0_samples(i, :);
    XCumuArr = XArr;

    for i = 0:delta:TH
        [simtimeArr, XArr, MArr] = sim_fn([i i+delta], XArr, [1], [1]);
        t = [t simtimeArr];
        XCumuArr = [XCumuArr; XArr];
    end
    plot(XCumuArr(:,1), XCumuArr(:,2))
end
end

function plotFun(~,X,currMode,plotOpts)
if nargin == 4
    plot(X(:,1),X(:,2),plotOpts);
else
    plot(X(:,1),X(:,2),'-r');
end

end

function drawProp()
cons_i = initConstraints();
cons_i_cube = cons_i.cube;
line([cons_i_cube(1,1) cons_i_cube(1,2) cons_i_cube(1,2) cons_i_cube(1,1) cons_i_cube(1,1)],[cons_i_cube(2,1) cons_i_cube(2,1) cons_i_cube(2,2) cons_i_cube(2,2) cons_i_cube(2,1)],'color','g');

cons_f = finalConstraints();
cons_f_cube = cons_f.cube;
line([cons_f_cube(1,1) cons_f_cube(1,2) cons_f_cube(1,2) cons_f_cube(1,1) cons_f_cube(1,1)],[cons_f_cube(2,1) cons_f_cube(2,1) cons_f_cube(2,2) cons_f_cube(2,2) cons_f_cube(2,1)],'color','g');
end

function initFun()
figure(1)
hold on

% illustrate porp 1(easy)
line([-1 -0.7 -0.7 -1 -1],[-6.5 -6.5 -5.6 -5.6 -6.5])



% figure(2)
% hold on
% figure(3)
% hold on
end

function cons = initConstraints()
% bigger initial set, easy to falsify
% initial = [-0.5 0.5 %x
%            -0.5 0.5 %y
%             ];

initial = [-0.4 0.4 %x
    -0.4 0.4 %y
    ];
cons.cube = initial;
cons.isCube = 1;
cons.mode = 1;
cons.isNonLinear = 0;
end

function cons = finalConstraints()

%% prop 1 (an easy one)
% too easy to violate using just 100 samples evolving for 10s, no
% segmentation required
% final = [-1 -0.7
%         -6.5 -5.8];

%% prop 2 (smaller)
% successfull with 50 sims
% final = [-1 -0.7
%         -6.5 -5.5];
%sys_opt.ScatterSim.MAX_SAMPLE_AGE = 0.7;
% sys_opt.ScatterSim.MAX_SWITCHINGS = inf;
% sys_opt.MODE_TIME_HORIZON = 0.1;

%% prop 3 (still smaller)
% successfull with 100 sims
%sys_opt.ScatterSim.MAX_SAMPLE_AGE = 0.7;
% sys_opt.ScatterSim.MAX_SWITCHINGS = inf;
% sys_opt.MODE_TIME_HORIZON = 0.1;

final = [-1 -0.7
    -6.5 -5.6];
cons.cube = final;
cons.isCube = 1;
cons.mode = -1;
cons.isNonLinear = 0;
end

function transitionMap = getTransitionMap(~)
%H = helper(NUM_STATE_VARS);

% guard = H.getConsMat([]);
% reset = H.getResetMap([]);
% transitionMap{1,1}{1} = H.getTrans(guard,reset);

transitionMap{1,1}{1} = [];

end

% easier than checking for every possible transition!
function modeTransitionMap = getModeTransitionMap()
% empty
modeTransitionMap = [];
end

function modeInvMap = getModeInvMap()
H = helper(NUM_STATE_VARS);
cons1 = H.getConstraint([X1 -1],10);
cons2 = H.getConstraint([X1 1],10);
cons3 = H.getConstraint([X2 -1],10);
cons4 = H.getConstraint([X2 1],10);

modeInvMap(1) = H.getConsMat([cons1 cons2 cons3 cons4]);

end

function modeDynMap = getModeDynMap()
    function Y = dyn(~,X,~,~)
        Y(1) = X(2);
        Y(2) = 5 * (1 - X(1)^2) * X(2) - X(1);
        Y = Y';
    end

modeDynMap(1).isNonLinear = 1;
modeDynMap(1).F = @dyn;

end

% approximate bounds on the state variables in every mode
function minStateVarsMap = getMinStateVarsMap()
for currMode = 1:NUM_MODES
    minStateVarsMap(currMode,:) = [-inf -inf ];
end
end

% approximate bounds on the state variables in every mode
function maxStateVarsMap = getMaxStateVarsMap()
for currMode = 1:NUM_MODES
    maxStateVarsMap(currMode,:) = [inf inf];
end
end


% approximate bounds on the dwell time in every mode
function  ret = getMaxDwellTimesMap()
% ignore the currmode arg
    function maxDwellTimesMap = retTimeBounds(~)
        maxDwellTimesMap = inf;
    end
ret = @retTimeBounds;
end

% approximate bounds on the dwell time in every mode
function  ret = getMinDwellTimesMap()
% ignore the currmode arg
    function minDwellTimesMap = retTimeBounds(~)
        minDwellTimesMap = 1;
    end
ret = @retTimeBounds;
end

%% helper funcs/ global vars

%% modes

function y = NUM_MODES()
y = 1;
end

function y = NUM_STATE_VARS()
y = 2;
end
function y = X1()
y = 1;
end
function y = X2()
y = 2;
end

