
clc
clear all

bag = rosbag('unist_outloop_ccw.bag');
bSel = select(bag,'Topic','/vehicle/state_est');
msgStructs = readMessages(bSel,'DataFormat','struct');

vy= cellfun(@(m) double(m.VLat),msgStructs);
% a= cellfun(@(m) double(m.a),msgStructs);  
vx = cellfun(@(m) double(m.VLong),msgStructs);
psi = cellfun(@(m) double(m.Psi),msgStructs);
df = cellfun(@(m) double(m.Df),msgStructs);
a_lat = cellfun(@(m) double(m.ALat),msgStructs);
lon = cellfun(@(m) double(m.Lon),msgStructs);
a_long = cellfun(@(m) double(m.ALong),msgStructs);
y = cellfun(@(m) double(m.Y),msgStructs);
mode = 'Real'
v = cellfun(@(m) double(m.V),msgStructs);
lat = cellfun(@(m) double(m.Lat),msgStructs);
x = cellfun(@(m) double(m.X),msgStructs);
% wz = cellfun(@(m) double(m.wz),msgStructs);
t = cellfun(@(m) double(m.Header.Stamp.Sec+(1e-9)*m.Header.Stamp.Nsec),msgStructs);
