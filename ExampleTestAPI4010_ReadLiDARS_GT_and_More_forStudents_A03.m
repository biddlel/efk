

% SAME original example, but using API version 3
% Menu does offer options for setting extra noises,etc.  Useful for testing project2/partA
% Search for the word NEW03, to see useful details


function main()

% MTRN4010.2024
% this example shows how to use the API for playing back a dataset.
% we also read sensors' events, we process the LiDAR events
% by converting scans (from onboard LiDAR1) from native representation
% ("scans") to range and intensity values. We also perform some conversion
% to cartesian representation, in the LiDAR CF
%we also create a figure, to show the Global Coordinate frame (Global CF , GCF)

%In addition, we create some  basic controls for operating the playBack
%session (buttons for resetting the session, for pausing/continuing and for ending.

% Questions: Ask the lecturer via forum or by email (j.guivant@unsw.edu.au)



clc; 

MyApi = APImtrn4010_v03();                      % init API  //NEW03


file = '.\Datasets\aDataUsr_007b.mat';
r = MyApi.b.LoadDataFile(file);                  % load dataset of interest

if r.ok<1, return ; end;                         % r.ok<1 : there are issues with the data file.
% possible issues:  file does not exist, or data structure is not correct. 

% you may exploit the returned info, which is about the length of the dataset
%nE=r.n(1) ;  % number of events in this dataset/playback session.
%nL=r.n(2) ;  % number of LiDAR events ....   : this may be useful to predefine certain Buffers we need in Project1.

%MyApi.b.Rst();                                   % API function to reset playback pointer  


%...........................................
%  Get "pointers" to useful functions, from API. We will call them many times and frequently.
ReadNextEvent=MyApi.RdE;                           % :read next event
ConvertRangestoXY=MyApi.b.ConvertRangestoXY;       % :convert scan ranges (polar) to carterian points in LiDAR's CF (*)
GetCurrentGT = MyApi.gcgt;                         % : get current actual pose.
AnimateShape = MyApi.AniShape;                     % : helper function for basic animation
GetRangeAndIntensityFromRawScan = MyApi.b.GetRangeAndIntensityFromRawScan;      % parses raw LidAR scans. (*)
% (*) : you must implement your own version, as part of project 1.


%...........................................

% Extra information (map of walls and poles in the area,etc) 
UsefulInfo=MyApi.b.GetInfo();   
%...........................................

GT = MyApi.b.GetGroundTruth();    
% API function "GetGroundTruth", it returns the full actual travelled trajectory, a.k.a. the ground truth.
% Those poses were taken at the same times of the LiDAR events.
% We use this info for validating accuracy of our solutions.

%--------------------------------------------------------
% Create a figure , to show, in Global CF, the full ground truth path; and
% also a handle to show the current GT position, dynamically during the playback session,
hhGCF=MkFigurePlotPoseAndGT(20,GT);

% Now, we plot some other static objects, such as walls and poles.
hold on ;  
Context=UsefulInfo.Context;
SomeBrownColor = [205, 127, 50]/255;  % RGB, sort of brown...
plot(Context.Walls(1,:),Context.Walls(2,:),'-','linewidth',2,'color',SomeBrownColor);       %plot walls 
plot(Context.Landmarks(1,:),Context.Landmarks(2,:),'m*');                                   %plot navigation poles.
hCar = plot(0,0,'r-');     % In that figure, add/create a line object to be used to animate the pose of the car in GCF.

% Other items may be added in Project 1   (such as OOIs in GCF, LiDAR scans in GCF, estimated platform pose, etc.)

hold off;                  % no more plots to be added in this figure. 

%--------------------------------------------------------
% create a figure and some graphical objects to dynamically plotting LiDAR1 scans, in LIDAR's CF (LiCF)
hh1=MkFigureToPlotLiDAR1sScan(21);
%--------------------------------------------------------
%...........................................
% Simple playback MENU   (you may use, it or you may create your own menu, so not needing to use this API function.) 
MyApi.b.MkMenuControl(20,150);  % here: ask API to offer a basic control menu in figure 20 (to allow "pause","end", "reset", "jump" playback sequence.
% NEW03   
%MyApi.b.MkMenuControl(figureNumber,MenuWidth_in_pixels);  

%MyApi.b.MkMenuControl(figureNumber);            
%original way, default MenuWidth_in_pixels = 110.
%...........................................
% To create menu, separate from you figures.
pause(0.2);
MyApi.b.MkSeparateMenuControl(55,300);   % NEW03     
%   MkSeparateMenuControl(figureNumber,width_in_pixels);    



%...........................................

% details about position and orientation of the LiDARs, in the platform. 
% (Not used in this example, but you will need them in Project1.)
Lidar1CFG =  UsefulInfo.LidarsCfg.Lidar1 ;    % installation info about Lidar#1
Lidar2CFG =  UsefulInfo.LidarsCfg.Lidar2 ;    % installation info about Lidar#2

pose0 = UsefulInfo.pose0 ;                    % platform's initial pose. 

UsefulInfo=[];  Context=[];% variables not needed anymore, in this example.


vw=[0;0];                       % program variable to store the last read dead recknoning measurements.
t0=0;                           % program variable to store the timestamp of last event.



%.............................................
% LOOP , asking for events, chronologically.
while 1,

e=ReadNextEvent();              % read next available event (event type, time and data)

t = double(e.t)*0.0001 ;        % time of current event, in seconds.
dt=t-t0 ; t0=t;                 % time elapsed since last event.  It will be useful in the implementations of certain processing, in Project 1.

% Dispatch actions, based on the type of sensor event.
switch(e.ty)
    
    case 0   % -----------------------------  "end of trip" event -------------------- 
        disp('Found the END') ; break ;   % data type = 0 --> END, no more events. end of trip. so END LOOP. 
    
    case 1   % -----------------------------  event due to LiDARs measurement
              % LiDAR event, ==> data will have a raw scan from LIDAr1, and anther scan from LiDAR2,
              d=e.d ;     % data is 301x2    [scan1,scan2], class uint16. d(:,1) is scan1, and d(:,2) is scan2
            
            % extract actual range and associated reflection intensities   
            [~,intensisties1,rangesM1] = GetRangeAndIntensityFromRawScan(d(:,1));
            %[~,intensisties2,rangesM2] = GetRangeAndIntensityFromRawScan(d(:,2));

            %[rangesCm,intensisties1,rangesM] = GetRangeAndIntensityFromRawScan(RawScan);
            % rangesCm: ranges in cm, class uint16
            % rangesM: ranges in metres, class single (a.k.a."float" in C/C++)
            % intensities: class uint8



            % ask API for current ground truth pose "get current ground truth"
            poseGTNow = GetCurrentGT();   % vehicle actual pose at the time of the LiDAR event

            % convert LiDARs' ranges (polar) to cartesian, in their local CF.
            [xx1,yy1]=ConvertRangestoXY(rangesM1,[]);        
            %[xx2,yy2]=ConvertRangestoXY(rangesM2,[]);
            
            % find which "pixels" are brilliant.
            ii1 = find(intensisties1>0);

            
            % ......................................................
            % refreh plots
              RefreshPLotsLiDAR1sScan(hh1,rangesM1,ii1,xx1,yy1);        % animate plots.


            % refresh, in figure, current ground truth position
            set(hhGCF(1),'xdata',poseGTNow(1),'ydata',poseGTNow(2));
            
            % refresh car shape having the current pose, in GCF.
            AnimateShape(hCar,1,poseGTNow);  
            % AnimateShape(handle to lineo bject ,shape number,pose);
            % helper function to animate basic shapes
            % note: shape number=1 is a triangle.
            % ......................................................

            
            
            
            % ****  NEW03  
            %pause(0.05);   % you can decide how long to pause, for fast or slow playback  
            
            % PAUSE may not be needed, as the API does it, if we tell it to do so, in the menu.
            % API does apply a default pause of 5 milliseconds, at each LiDAR event.
            

            

    case 2 % -----------------------------  Event due to dead-reckoning measurement
        % dead reckoning (longitudinal velocity (aka speed) and angular rate)
            vw=e.d;  % 2x1,  [ v ; w ]

    otherwise %-----------------------------
        % ? % we do not consider other sensors in Project 1, so we ignore those. 
end    

end     % end loop

MyApi.b.PrintStsA(0);   % print some statistics, for the teaching staff.
end     % end main()

%-----------------------------------------------------------------
function h=MkFigurePlotPoseAndGT(ff,GTposes)
    figure(ff); clf();
    x=GTposes(1,:);
    y=GTposes(2,:);
    plot(x,y,'g');                   % static plot: the GT path.
    hold on;
    h = plot(0,0,'r*');              % to be used for animating the vehicle current position
    grid on ; axis equal ;
    axis([-5,20,-5,20]);
    xlabel('X'); ylabel('Y');
    title('Global CF');
end
%-----------------------------------------------------------------


%......................................................................
% parse raw range values
% function [r,i] = GetRangeAndIntensityFromRawScan(scan)
% Students do this.
% end
% -----------------------------------------------------------------------------------------------------
% function in which I create some plots, some of them to be updated dynamically
function hh=MkFigureToPlotLiDAR1sScan(figu)
    figure(figu); clf();
    aa =  ((0:300)*0.5)-75 ;
    subplot(211) ; h1=plot(aa,aa*0,'.b'); axis([-75,75,-10,30]); hold on ; 
    h2=plot(0,0,'+r');  % to show the "brilliant points"
    title('LiDAR_1 (in polar)');  grid on ;
    ylabel('range (m)')
    xlabel('azimuth (degrees)')
    subplot(212) ; h3=plot(0,0,'.b'); axis([-5,55,-30,30]); grid on ;  title('LiDAR_1 (pointing ahead)'); xlabel('X   :   Ahead ==>   ');  %axis equal ;
    hh=[h1,h2,h3];   % handles to graphical objects 
end

%......................................................................
% refresh LiDAR scan in figure, Local CF, ...etc.
function RefreshPLotsLiDAR1sScan(hh,r,ii,xx,yy)
h1 = hh(1); h2 = hh(2);  h3 = hh(3);
% polar
set(h1,'ydata',r);                      % all raw points,
aa=single(ii-1)*0.5-75;
set(h2,'xdata',aa,'ydata',r(ii));       % selected ones, using indexes in array ii    

% Cartesian
set(h3,'xdata',xx,'ydata',yy);     
end
%......................................................................

