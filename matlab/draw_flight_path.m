function draw_flight_path(block)
% Simulation-only sink; collect at 20 Hz and draw on normal termination.
block.NumDialogPrms=2;
block.NumInputPorts=1;
block.NumOutputPorts=0;
block.SetPreCompInpPortInfoToDynamic;
block.InputPort(1).Dimensions=[3 1];
block.InputPort(1).DirectFeedthrough=true;
block.SampleTimes=[40*block.DialogPrm(2).Data 0];
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('Start',@start);
block.RegBlockMethod('Outputs',@sample);
block.RegBlockMethod('Terminate',@finish);
end

function start(b)
if b.DialogPrm(1).Data, points('start',b.BlockHandle,[]); end
end
function sample(b)
if b.DialogPrm(1).Data, points('add',b.BlockHandle,b.InputPort(1).Data(:).'); end
end
function finish(b)
if ~b.DialogPrm(1).Data, return; end
xyz=points('finish',b.BlockHandle,[]);
if isempty(xyz), return; end
figure('Name','DroneSim flight path');
plot3(xyz(:,1),xyz(:,2),xyz(:,3)); grid on; axis equal;
xlabel('East (m)'); ylabel('North (m)'); zlabel('Height (m)');
end
function out=points(action,key,value)
persistent traces
if isempty(traces), traces=containers.Map('KeyType','double','ValueType','any'); end
out=[];
switch action
    case 'start', traces(key)=zeros(0,3);
    case 'add', traces(key)=[traces(key);value];
    case 'finish', if isKey(traces,key), out=traces(key); remove(traces,key); end
end
end
