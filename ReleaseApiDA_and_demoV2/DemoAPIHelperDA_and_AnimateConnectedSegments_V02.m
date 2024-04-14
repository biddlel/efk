function DemoAPIHelperDA_and_AnimateConnectedSegments()
%--------------------------------------------------------------------
% Example about using API DA helper function.
%  If you DA from Project1 is not working well you may use this helper function, in Project2.
%  MTRN4010
% run it from a folder in which you have the API and the dataset used in
% this example.
% J.E.G. MTRN4010/2024.
%--------------------------------------------------------------------

clc();
MyApi = APImtrn4010_v04();                      % init API

% To test the DA in similar conditions to those of your project, we use the map
% from this dataset.

file = '.\Datasets\aDataUsr_007b.mat';
r = MyApi.b.LoadDataFile(file);                  % load dataset of interest
if r.ok<1, return ; end;                         % r.ok<1 : there are issues with the data file.



% 2 helper functions
ExePlotSegments = MyApi.b.ShSe ;                % this helper will be useful for visualization of DA and etc. 
MyDA =  MyApi.b.DA.MyDA ;                       % Data aAssociation helpt function.
disp(MyApi.b.DA.hlp());     % print help about DA




% -------------------------------------------
figure(1) ; clf();
Landmarks=GetAndPlotInFo(MyApi);
hold on ;
hppB = plot(0,0,'+k');
hs = plot(0,0,'-g','linewidth',2) ;  % for segments: some color, some style, some thickness, etc (you decide)



for i=1:1000.

ppB = Landmarks(:,1:3:end) ;                        % some subset of Landmarks  
ppB(:,3) = [10;10] ;                                % force an intruder, not a landmark.
ppB = ppB +   (rand(size(ppB))-0.5)*2*1.3+0.1;      % "random jumps nearby"

set(hppB,'xdata',ppB(1,:),'ydata',ppB(2,:));

[na,~,iiO,uuL]=MyDA(ppB(1,:),ppB(2,:),Landmarks(1,:),Landmarks(2,:),1.5);

if (na>0), 
    % points ppB(:,iiO) are associated with Landmarks(:,uuL)
    ExePlotSegments(hs,ppB(:,iiO),Landmarks(:,uuL));
else
    ExePlotSegments(hs,[],[]);
end    
   

pause(0.15);

end
end

%--------------------------------------------------------
% Just some static plots of the context, etc.
function Landmarks=GetAndPlotInFo(MyApi)
% Extra information (map of walls and poles in the area,etc) 
UsefulInfo=MyApi.b.GetInfo();   
% Create a figure , to show, in Global CF, the full ground truth path; and
% also a handle to show the current GT position, dynamically during the playback session,
% Now, we plot some other static objects, such as walls and poles.
hold on ;  
Context=UsefulInfo.Context;
SomeBrownColor = [205, 127, 50]/255;  % RGB, sort of brown...
Landmarks=Context.Landmarks;
plot(Context.Walls(1,:),Context.Walls(2,:),'-','linewidth',2,'color',SomeBrownColor);       %plot walls 
plot(Landmarks(1,:),Landmarks(2,:),'m*');                                   %plot navigation poles.
axis([-5,20,-5,20]);  xlabel('X'); ylabel('Y');  title('testing my DA');
end
%--------------------------------------------------------    
%------------------------ END of Example ----------------------------------