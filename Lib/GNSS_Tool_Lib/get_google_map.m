function [XX YY M Mcolor] = get_google_map(lat, lon, varargin)
%% Gets a google map using the Google Static Maps API
% 
% REQUIREMENTS:
% 1)  deg2utm() (Conversion from degrees to UTM coordinates, found on
% MATLAB Central. 
%
% 2)  THIS FUNCTION REQUIRES A GOOGLE MAPS API KEY! (READ ON)
% To query and return a Google Map using the Google Static Map API, Google
% requires that you have a valid MAPS API KEY. The key is free and only
% requires a Google account and a valid domain (e.g. http://ccom.unh.edu )
% from which your queries will come. Note, this last requirement places
% serious limitations on the portability of this function, as it is
% necessarily tied to a domain and therefore (usually) a physical location.
% 
% A Google Maps API Key may be obtained here: 
% http://code.google.com/apis/maps/signup.html
%
% Once you obtain your key, replace the text assigned to the MAPS_API_KEY
% variable below with your own.
%
% USAGE:
%
% [XX YY M Mcolor] = get_google_map( latitude, longitude, ...
%                              Property, Value,...
%                              Property, Value)
%
% PROPERTIES:
%    Height (640)      Height of the image in pixels (max 640)
%    Width  (640)      Width of the image in pixels (max 640)
%    Zoom (15)         Zoom Level (1-19) Zoom of 15 = ~3.4m/pixel.
%    MapType ('satellite')  Type of map to return. Any of [roadmap, mobile,
%                           satellite, terrain, hybrid, mapmaker-roadmap,
%                           mapmaker-hybrid) See the Google Maps API for
%                           more information.
%    Marker            The marker argument is a text string with fields
%                      conforming to the Google Maps API. The following are
%                      valid examples:
%                      '43.0738740,-70.713993' (dflt midsize orange marker)
%                      '43.0738740,-70.713993,blue' (midsize blue marker)
%                      '43.0738740,-70.713993,yellowa' (midsize yellow
%                      marker with label "A")
%                      '43.0738740,-70.713993,tinyredb' (tiny red marker
%                      with label "B")
%    APIKey           - (string) set your own API key which you obtained from Google: 
%                     http://developers.google.com/maps/documentation/staticmaps/#api_key
%                     This will enable up to 25,000 map requests per day, 
%                     compared to a few hundred requests without a key. 
%                     To set the key, use:
%                     get_google_map('APIKey','SomeLongStringObtaindFromGoogle')
%                     You need to do this only once to set the key.
%                     To disable the use of a key, use:
%                     get_google_map('APIKey','') [This option is borrowed
%                     from plot_google_map(), with many thanks.]
%
% RESULTS:
%    XX                Estimate of Easting pixel coordinates in UTM
%    YY                Estimate of Northing pixel coordinates in UTM
%    M                 Image data matrix (height x width)
%    Mcolor            Image colormap
%
% EXAMPLE:
%
%    % Get the map
%    [XX YY M Mcolor] = get_google_map(43.0738740,-70.713993);
%    % Plot the result
%    imagesc(XX,YY,M); shading flat;
%    colormap(Mcolor)
%    
% Acknowledgements:
% Zohar Bar-Yehuda for his submission of plot_google_map() which is, in
% many ways, a great improvement on this effort. In particular, his
% treatment of the Google Maps API has been reproduced here. 
%
% References:
% https://developers.google.com/maps/documentation/static-maps/
% http://www.mathworks.com/matlabcentral/fileexchange/27627-zoharby-plot-google-map
%
% KNOWN BUGS AND ISSUES:
% 1) MAP Bounds. The bounds reported by this function are an approximation
% based on repeated trials, as there is no way (to my knowledge) in which
% to query Google for the bounds of the map image exactly. This method is
% potentially ripe with errors. For example, it may not produce correct
% bounds at latitudes far from 40 degrees N/S where it was calibrated. 
% 2) No provision has been programmed to handle locations near the date
% line, nor boundaries between UTM zones. Images that cross these will
% likely produce improper bounds.
% 
% * Val Schmidt
% * University of New Hampshire
% * Center for Coastal and Ocean Mapping
% * 2009, 2016
persistent apiKey
if isnumeric(apiKey)
    % first run, check if API key file exists
    if exist('api_key.mat','file')
        load api_key
    else
        apiKey = '';
    end
end
cal_distance = 30;  % in degrees of latitude
degpermeter = 1/60*1/1852;
% HANDLE ARGUMENTS
height = 640;
width = 640;
zoomlevel = 15;
maptype = 'satellite';
markeridx = 1;
markerlist = {};
if nargin > 2
    for idx = 1:2:length(varargin)
        switch varargin{idx}
            case 'Height'
                height = varargin{idx+1};
            case 'Width'
                width = varargin{idx+1};
            case 'Zoom'
                zoomlevel = varargin{idx+1};
            case 'MapType'
                maptype = varargin{idx+1};
            case 'Marker'
                markerlist{markeridx} = varargin{idx+1};
                markeridx = markeridx + 1;
            case 'apikey'
                apiKey = varargin{idx+1}; % set new key
                % save key to file
                funcFile = which('get_google_map.m');
                pth = fileparts(funcFile);
                keyFile = fullfile(pth,'api_key.mat');
                save(keyFile,'apiKey')
            otherwise
                error(['Unrecognized variable: ' varargin{idx}])
        end
    end
end
if zoomlevel <1 || zoomlevel > 19
    error('Zoom Level must be > 0 and < 20.')
end
if mod(zoomlevel,1) ~= 0
    zoomlevel = round(zoomlevel)
    warning(['Zoom Level must be an integer. Rounding to '...
        num2str(zoomlevel)]);
end
% CONSTRUCT QUERY URL
preamble = 'http://maps.googleapis.com/maps/api/staticmap';
location = ['?center=' num2str(lat,10) ',' num2str(lon,10)];
cal_location = ['?center=' num2str(lat - degpermeter*cal_distance,10) ',' num2str(lon,10)];
zoom = ['&zoom=' num2str(zoomlevel)];
size = ['&size=' num2str(width) 'x' num2str(height)];
maptype = ['&maptype=' maptype ];
markers = '&markers=';
for idx = 1:length(markerlist)
    if idx < length(markerlist)
            markers = [markers markerlist{idx} '%7C'];
    else
            markers = [markers markerlist{idx}];
    end
end
format = '&format=png';
if ~isempty(apiKey)
    key = ['&key=' apiKey];
else
    key = '';
end
sensor = '&sensor=false';
url = [preamble location zoom size maptype format markers sensor key];
cal_url = [preamble cal_location zoom size maptype format markers sensor key];
% GET THE IMAGE
[M Mcolor] = webread(url);
M = cast(M,'double');
% ESTIMATE BOUNDS OF IMAGE:
% We get 2 images instead of just one, separated by a known distance. Then
% we cross-correlate one with the other to find the distance between the
% two images in pixels. Divide one by the other and you have distance per
% pixel. From this and the center point of we can calculate the coordinates
% of each. 
% GET THE CAL IMAGE
[Mcal Mcolorcal] = webread(cal_url);
Mcal = cast(Mcal,'double');
% Cross correlate a column in the middle of the data to get the shift
% between them in pixels.
comparecol = floor(width/2);
c = xcorr(M(20:end,comparecol),Mcal(20:end,comparecol)); % skip first 20 pixels
c = fftshift(c);
[val, pixels] = max(c(1:floor(end/2)));
dx = cal_distance/pixels;
% Convert coordinates to UTM.
[lonutm, latutm, zone] = deg2utm(lat,lon);
L = dx*(width-1);
W = dx*(height-1);
XX = 0:dx:L;
YY = 0:dx:W;
XX = XX - mean(XX) + lonutm;
YY = YY - mean(YY) + latutm;
