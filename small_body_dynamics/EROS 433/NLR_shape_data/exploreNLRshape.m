clear all; close all; clc;

shapemap = fitsread('shapemap_r4.fit');

%    The map is in the form of a binary table with              
%    one row for each 0.125-degree latitude, from 0 to 360 degrees             
%    East longitude and 90 to -90 degrees latitude.
stride=.125;

lats = 90:-stride:-90+stride;
lons = 0:stride:360-stride;

lats = lats*pi/180;
lons = lons*pi/180;

assert(size(shapemap,1)==length(lats))
assert(size(shapemap,2)==length(lons))

% We show this in xyz. TO get xyz, we have that x = r*cos(lat)*cos(lon),
% y=r*cos(lat)*sin(lon), z=r*sin(lat)

% Yes, we could vectorize. But this is exploratory.
decimate_lon = 10;
decimate_lat = 10;
xyzpoints = zeros(size(shapemap,1)*size(shapemap,2),3);
xyzidx = 1;
for lonix=1:decimate_lon:length(lons)
    for latix=1:decimate_lat:length(lats)
        r = shapemap(latix,lonix);
        lat = lats(latix);
        lon = lons(lonix);
        x = r*cos(lat)*cos(lon);
        y = r*cos(lat)*sin(lon);
        z = r*sin(lat);
        xyzpoints(xyzidx,:) = [x,y,z];
        xyzidx = xyzidx + 1;
    end
end

xvec = xyzpoints(:,1);
yvec = xyzpoints(:,2);
zvec = xyzpoints(:,3);

%% Simple plot3
plot3(xyzpoints(:,1),xyzpoints(:,2),xyzpoints(:,3),'.');