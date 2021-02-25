%% Edge generation
%     argument?F
%         Self-estimated series?Fxlist1, xlist2
%         Landmark series?Fzlist1, zlist2
%         Observation number?Ft1, t2
%    Return value?F
%         Edge information structure
function edge = make_edge(xlist1, xlist2, zlist1, zlist2, t1, t2)
global sigma
edge = struct;
% zlist1
% zlist2
% Extraction
yaw1 = xlist1(3);
yaw2 = xlist2(3);
angle1 = zlist1(2);
angle2 = zlist2(2);

% World coordinate system angle calculation (noise-contaminated phi)
tangle1 = pi2pi(yaw1 + angle1);
tangle2 = pi2pi(yaw2 + angle2);

% Extraction
x1 = xlist1(1);
y1 = xlist1(2);
x2 = xlist2(1);
y2 = xlist2(2);
d1 = zlist1(1);
d2 = zlist2(1);

% World coordinate system landmark position
tdx1 = x1 + d1*cos(tangle1);
tdy1 = y1 + d1*sin(tangle1);
tdx2 = x2 + d2*cos(tangle2);
tdy2 = y2 + d2*sin(tangle2);

hyaw = zlist1(3) - zlist2(3) + angle1 - angle2;

% Calculate the deviation between two observations
edge.e = [tdx2; tdy2] - [tdx1; tdy1];
% edge.e(3) = pi2pi(yaw2 - yaw1 - hyaw);
edge.e(3) = pi2pi(yaw2 - yaw1 - zlist1(3) + zlist2(3));

% Rotation matrix creation
R1 = [cos(tangle1), -sin(tangle1), 0;
    sin(tangle1), cos(tangle1), 0
    0, 0, 1];
R2 = [cos(tangle2), -sin(tangle2), 0;
    sin(tangle2), cos(tangle2), 0
    0, 0, 1];

sig1 = sigma;
sig2 = sigma;

edge.OMEGA = inv(R1*sig1*R1' + R2*sig2*R2');
[edge.d1,edge.d2] = deal(d1,d2);
[edge.yaw1,edge.yaw2] = deal(yaw1,yaw2);
[edge.angle1,edge.angle2] = deal(angle1,angle2);
[edge.t1,edge.t2] = deal(t1,t2);


 
