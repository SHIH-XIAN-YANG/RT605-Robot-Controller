syms rx ry rz dx dy dz Fx Fy Fz Tx Ty Tz

Rtcp_sensor = transpose(rotz(rz)*roty(ry)*rotx(rx));
Tmat_sensor_tcp = [transpose(Rtcp_sensor), [dx; dy; dz]; [0, 0, 0, 1]];
Tmat_tcp_sensor = inv(Tmat_sensor_tcp);
Porg_tcp_sensor = Tmat_tcp_sensor(1:3, 4); 
pz = Porg_tcp_sensor(3);
py = Porg_tcp_sensor(2);
px = Porg_tcp_sensor(1);
P_by = [0, -pz, py; pz, 0, -px; -py, px, 0];
Fts = [Fx; Fy; Fz; Tx; Ty; Tz];
FTtcp_sensor = [Rtcp_sensor, zeros(3, 3); [P_by*Rtcp_sensor, Rtcp_sensor]]*Fts;

% function_FTtcp_sensor = matlabFunction(FTtcp_sensor, 'File', 'StaticForceTransform_FtsToTcp');
